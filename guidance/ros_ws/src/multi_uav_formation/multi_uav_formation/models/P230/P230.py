#! /usr/bin/env python3

import numpy as np
import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest
from geographic_msgs.msg import GeoPointStamped

from Utils import *


POSITION_IGNORE = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ
VELOCITY_IGNORE = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ
ACCELERATION_IGNORE = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ
YAW_IGNORE = PositionTarget.IGNORE_YAW
YAW_RATE_IGNORE = PositionTarget.IGNORE_YAW_RATE

POSITION_YAW = VELOCITY_IGNORE + ACCELERATION_IGNORE + YAW_RATE_IGNORE
VELOCITY_YAW = POSITION_IGNORE + ACCELERATION_IGNORE + YAW_RATE_IGNORE
ACCELERATION_YAW = POSITION_IGNORE + VELOCITY_IGNORE + YAW_RATE_IGNORE

POSITION_YAW_RATE = VELOCITY_IGNORE + ACCELERATION_IGNORE + YAW_IGNORE
VELOCITY_YAW_RATE = POSITION_IGNORE + ACCELERATION_IGNORE + YAW_IGNORE
ACCELERATION_YAW_RATE = POSITION_IGNORE + VELOCITY_IGNORE + YAW_IGNORE

POSITION_SETPOINT_ALL_IGNORE = POSITION_IGNORE + VELOCITY_IGNORE + ACCELERATION_IGNORE + YAW_IGNORE + YAW_RATE_IGNORE

ANGLE_RATE_IGNORE = AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_ROLL_RATE + AttitudeTarget.IGNORE_YAW_RATE
ATTITUDE_IGNORE = ANGLE_RATE_IGNORE + AttitudeTarget.IGNORE_ATTITUDE

ATTITUDE_SETPOINT_ALL_IGNORE = ATTITUDE_IGNORE + ANGLE_RATE_IGNORE + AttitudeTarget.IGNORE_THRUST


class P230():
    def __init__(self, name='uav1') -> None:
        self.name = name

        self.armService = rospy.ServiceProxy(f'/{name}/mavros/cmd/arming', CommandBool)
        self.setModeService = rospy.ServiceProxy(f'/{name}/mavros/set_mode', SetMode)
        self.takeoffService = rospy.ServiceProxy(f'/{name}/mavros/cmd/takeoff', CommandTOL)
        self.landService = rospy.ServiceProxy(f'/{name}/mavros/cmd/land', CommandTOL)

        self.meState = State()

        self.mePositionENU = np.zeros(3)
        self.meVelocityENU = np.zeros(3)
        self.meSpeed = 0
        self.meAccelerationFLU = np.zeros(3)
        self.meAccelerationENU = np.zeros(3)
        self.meQuaternionENU = np.array([0, 0, 0, 1])
        self.meRPYRadENU = np.zeros(3)
        self.meRPYRadNED = np.zeros(3)
        self.meRPYDegENU = np.zeros(3)

        self.stateSub = rospy.Subscriber(f'/{name}/mavros/state', State, self.state_cb)
        self.localPosSub = rospy.Subscriber(f'/{name}/mavros/local_position/pose', PoseStamped, self.local_pos_cb)
        self.velSub = rospy.Subscriber(f'/{name}/mavros/local_position/velocity_local', TwistStamped, self.vel_cb)
        self.imuSub = rospy.Subscriber(f'/{name}/mavros/imu/data', Imu, self.imu_cb)
        self.gpsOriginPub = rospy.Publisher(f'/{name}/mavros/global_position/set_gp_origin', GeoPointStamped,queue_size=1)
        self.gpsOriginSub = rospy.Subscriber(f'/{name}/mavros/global_position/gp_origin', GeoPointStamped, self.origin_cb)
        
        self.gpsOriginZurichIrchelPark = GeoPointStamped()
        self.gpsOriginZurichIrchelPark.position.latitude = 47.397742
        self.gpsOriginZurichIrchelPark.position.longitude = 8.545594
        self.gpsOriginZurichIrchelPark.position.altitude = 535.4
        self.gpsOriginZurichIrchelPark.header.frame_id = "base_link"
        self.gpsOrigin = self.gpsOriginZurichIrchelPark

        self.meGpsOrigin = GeoPointStamped()

        self.setpoint = PositionTarget()
        self.setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.setpoint.type_mask = POSITION_YAW_RATE
        self.setpoint.position.x = 0
        self.setpoint.position.y = 0
        self.setpoint.position.z = 0
        self.setpointPub = rospy.Publisher(f'/{name}/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

        self.attitudeSetpoint = AttitudeTarget()
        self.attitudeSetpoint.type_mask = ATTITUDE_IGNORE
        self.attitudeSetpoint.orientation.x = 0
        self.attitudeSetpoint.orientation.y = 0
        self.attitudeSetpoint.orientation.z = 0
        self.attitudeSetpoint.orientation.w = 1
        self.attitudeSetpointPub = rospy.Publisher(f'/{name}/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

        self.velMax = np.array([1, 1, 1])
        self.accMax = np.array([1, 1, 1])
        self.yawRateRadMax = 1
        self.yawRadNED = 0
        self.rollDegMax = 10.0
        self.pitchDegMax = 10.0
        self.rpRadMax = np.deg2rad(np.array([self.rollDegMax, self.pitchDegMax]))

        self.rollOffsetRad = 0.0
        self.pitchOffsetRad = 0.0

        self.fcu_url = None
        while not self.fcu_url:
            self.fcu_url = rospy.get_param(f'/{name}/mavros/fcu_url')
            time.sleep(1)
        print(f'Connected to FCU at {self.fcu_url}')
        if 'udp' in self.fcu_url:
            self.mode = 'sim'
        elif 'dev' in self.fcu_url:
            self.mode = 'real'
        else:
            self.mode = 'unknown'
            raise ValueError('FCU URL parse failed')

        self.hoverThrottle = 0.4

        self.kp = 0.5

        self.positionThreshold = 0.5 if self.mode == 'sim' else 0.1
        self.speedThreshold = 0.4 if self.mode == 'sim' else 0.1 

        print('P230 node initialized')

    def state_cb(self, msg):
        self.meState = msg

    def local_pos_cb(self, msg):
        self.mePositionENU = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.meQuaternionENU = np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        self.meRPYRadENU = quaternion2euler(self.meQuaternionENU)
        self.meRPYRadNED = rpyENU2NED(self.meRPYRadENU)
        self.meRPYDegENU = np.rad2deg(self.meRPYRadENU)

    def vel_cb(self, msg):
        self.meVelocityENU = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        self.meSpeed = np.linalg.norm(self.meVelocityENU)

    def imu_cb(self, msg):
        self.meAccelerationFLU = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.meAccelerationENU = flu2enuRotationMatrix(self.meRPYRadENU[0], self.meRPYRadENU[1], self.meRPYRadENU[2]) @ self.meAccelerationFLU + np.array([0, 0, -GRAVITY])

    def origin_cb(self, msg):
        self.meGpsOrigin = msg

    def intoOffboardMode(self):
        try:
            offboardSetMode = SetModeRequest()
            offboardSetMode.custom_mode = 'OFFBOARD'
            response = self.setModeService.call(offboardSetMode)

            if not response:
                rospy.logerr("into offboard mode failed!")
                return False

            rospy.loginfo("into offboard mode successful!")
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return False

    def arm(self):
        try:
            armCmd = CommandBoolRequest()
            armCmd.value = True
            response = self.armService.call(armCmd)

            if not response:
                rospy.logerr("arm failed!")
                return False

            rospy.loginfo("arm successful!")
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return False

    def disarm(self):
        try:
            armCmd = CommandBoolRequest()
            armCmd.value = False
            response = self.armService.call(armCmd)

            if not response.result:
                rospy.logerr("disarm failed!")
                return False

            rospy.loginfo("disarm successful!")
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return False

    def takeoff(self, height=1.0):
        self.takeoffCmd = CommandTOLRequest()
        self.takeoffCmd.altitude = height
        return self.takeoffService.call(self.takeoffCmd)

    def land(self):
        self.landCmd = CommandTOLRequest()
        return self.landService.call(self.landCmd)

    def isArmed(self):
       return self.meState.armed

    def mode(self):
        return self.meState.mode

    def sendHeartbeat(self):
        if self.attitudeSetpoint.type_mask == ANGLE_RATE_IGNORE:
            self.attitudeSetpointPub.publish(self.attitudeSetpoint)
        else:
            self.setpointPub.publish(self.setpoint)
        self.printControl()

    def setPositionControlMode(self):
        self.setpoint.type_mask = POSITION_YAW
        self.attitudeSetpoint.type_mask = ATTITUDE_SETPOINT_ALL_IGNORE

    def setVelocityControlMode(self):
        self.setpoint.type_mask = VELOCITY_YAW
        self.attitudeSetpoint.type_mask = ATTITUDE_SETPOINT_ALL_IGNORE

    def setAccelerationControlMode(self):
        self.setpoint.type_mask = ACCELERATION_YAW
        self.attitudeSetpoint.type_mask = ATTITUDE_SETPOINT_ALL_IGNORE

    def setAttitudeControlMode(self):
        self.setpoint.type_mask = POSITION_SETPOINT_ALL_IGNORE
        self.attitudeSetpoint.type_mask = ANGLE_RATE_IGNORE

    def printControl(self):
        print('-' * 10 + 'Control' + '-' * 10)
        print('Control Mode: ', end='')
        if self.attitudeSetpoint.type_mask == ANGLE_RATE_IGNORE:
            print('Attitude')
            quat = self.attitudeSetpoint.orientation
            print(f'Quat: ({quat.x:.2f}, {quat.y:.2f}, {quat.z:.2f}, {quat.w:.2f})')
            print(f'Euler: {rpyString(quaternion2eulerXYZW(quat))}')
            print(f'Thrust: {self.attitudeSetpoint.thrust:.4f}')
        elif self.setpoint.type_mask == POSITION_YAW:
            print('Position & Yaw')
            print(f'Position: {pointString(self.setpoint.position)}')
            print(f'Yaw: {np.rad2deg(self.setpoint.yaw):.2f} deg')
        elif self.setpoint.type_mask == VELOCITY_YAW:
            print('Velocity & Yaw')
            print(f'Velocity: {pointString(self.setpoint.velocity)}')
            print(f'Yaw: {np.rad2deg(self.setpoint.yaw):.2f} deg')
        elif self.setpoint.type_mask == ACCELERATION_YAW:
            print('Acceleration & Yaw')
            print(f'Acceleration: {pointString(self.setpoint.acceleration_or_force)}')
            print(f'Yaw: {np.rad2deg(self.setpoint.yaw):.2f} deg')
        elif self.setpoint.type_mask == POSITION_YAW_RATE:
            print('Position & Yaw Rate')
            print(f'Position: {pointString(self.setpoint.position)}')
            print(f'Yaw rate: {np.rad2deg(self.setpoint.yaw_rate):.2f} deg')
        elif self.setpoint.type_mask == VELOCITY_YAW_RATE:
            print('Velocity & Yaw Rate')
            print(f'Velocity: {pointString(self.setpoint.velocity)}')
            print(f'Yaw rate: {np.rad2deg(self.setpoint.yaw_rate):.2f} deg')
        elif self.setpoint.type_mask == ACCELERATION_YAW_RATE:
            print('Acceleration & Yaw Rate')
            print(f'Acceleration: {pointString(self.setpoint.acceleration_or_force)}')
            print(f'Yaw rate: {np.rad2deg(self.setpoint.yaw_rate):.2f} deg')
        else:
            print('Unknown')

    def printMe(self):
        print('-' * 10 + 'Me' + '-' * 10)
        print('Mode: ', self.meState.mode)
        print('Position ENU: ', arrayString(self.mePositionENU))
        print('Velocity ENU: ', arrayString(self.meVelocityENU))
        print(f'Speed: {self.meSpeed:.2f}')
        print('Acceleration FLU: ', arrayString(self.meAccelerationFLU))
        print('Acceleration ENU: ', arrayString(self.meAccelerationENU))
        print('Euler Deg ENU: ', rpyString(self.meRPYRadENU))
        print(f'Hover throttle: {self.hoverThrottle:.2f}')

    def distanceToPointENU(self, pointENU):
        return np.linalg.norm(self.mePositionENU - pointENU)

    def nearPositionENU(self, pointENU, tol=None):
        if tol is None:
            tol = self.positionThreshold
        return self.distanceToPointENU(pointENU) <= tol

    def nearSpeed(self, speed, tol=None):
        if tol is None:
            tol = self.speedThreshold
        return abs(speed - self.meSpeed) <= tol

    def aboveHeight(self, height):
        return self.mePositionENU[2] >= height

    def belowHeight(self, height):
        return self.mePositionENU[2] <= height

    def positionENUControl(self, posENU, yawRadENU):
        self.setPositionControlMode()
        self.setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.setpoint.position.x = posENU[0]
        self.setpoint.position.y = posENU[1]
        self.setpoint.position.z = posENU[2]
        self.setpoint.yaw = yawRadENU

    def saturateVelocity(self, velENU):
        return np.clip(velENU, -self.velMax, self.velMax)

    def velocityENUControl(self, velENU, yawRadENU):
        self.setVelocityControlMode()
        self.setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        velENU = self.saturateVelocity(velENU)
        self.setpoint.velocity.x = velENU[0]
        self.setpoint.velocity.y = velENU[1]
        self.setpoint.velocity.z = velENU[2]
        self.setpoint.yaw = yawRadENU

    def saturateAccleration(self, accENU):
        return np.clip(accENU, -self.accMax, self.accMax)

    def accerlationENUControl(self, accENU, yawRadENU):
        self.setAccelerationControlMode()
        self.setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        accENU = self.saturateAccleration(accENU)
        self.setpoint.acceleration_or_force.x = accENU[0]
        self.setpoint.acceleration_or_force.y = accENU[1]
        self.setpoint.acceleration_or_force.z = accENU[2]
        self.setpoint.yaw = yawRadENU

    def saturateAttitude(self, rpyRad):
        return np.concatenate((np.clip(rpyRad[:2], -self.rpRadMax, self.rpRadMax), rpyRad[2:]))

    def rpyENUThrustControl(self, rpyRadENU, thrust):
        self.setAttitudeControlMode()
        rpyRadENU = self.saturateAttitude(rpyRadENU)
        rpyRadENU[0] += self.rollOffsetRad
        rpyRadENU[1] += self.pitchOffsetRad
        controlQuaternion = euler2quaternion(rpyRadENU)
        self.attitudeSetpoint.thrust = thrust
        self.attitudeSetpoint.orientation.x = controlQuaternion[1]
        self.attitudeSetpoint.orientation.y = controlQuaternion[2]
        self.attitudeSetpoint.orientation.z = controlQuaternion[3]
        self.attitudeSetpoint.orientation.w = controlQuaternion[0]

    def constantVelocityLineENUControl(self, constantVelocityENU, positionENU, yawRadENU):
        kI = 0.05
        kp = 0.5
        self.setVelocityControlMode()
        self.velocityENUControl(constantVelocityENU + kI * (positionENU - self.mePositionENU) + kp * (self.meVelocityENU - constantVelocityENU), yawRadENU)

    def hoverWithYaw(self, yawRadENU):
        self.velocityENUControl([0, 0, 0], yawRadENU)

    def velocityToPointENUControl(self, pointENU, yawRadENU):
        velENU = self.kp * (pointENU - self.mePositionENU)
        self.velocityENUControl(velENU, yawRadENU)
        
    def setGpsOrigin(self, origin: GeoPointStamped = None):
        if origin is None:
            origin = self.gpsOrigin

        origin.header.stamp = rospy.Time.now()

        self.gpsOriginPub.publish(origin)
        rospy.loginfo(f"Setting GPS global origin to: {origin.position.latitude}, {origin.position.longitude}, {origin.position.altitude}")

    def hasSetOrigin(self) -> bool:
        return self.gpsOrigin.position.latitude == self.meGpsOrigin.position.latitude