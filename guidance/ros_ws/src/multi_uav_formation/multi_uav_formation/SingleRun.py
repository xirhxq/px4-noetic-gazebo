#!/usr/bin/env python3

import argparse
import builtins
import copy
import datetime
import json
import os
import pickle
import sys
import time
import threading

import numpy as np
import rospy

from collections import deque

utils_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__)))
sys.path.append(utils_path)
sys.path.append(src_path)
print(sys.path)

from models.P230.P230 import P230
from Communicator import Communicator

from Utils import *
from State import State

def stepEntrance(method):
    def wrapper(self, *args, **kwargs):
        self.stateStartTime = self.getTimeNow()
        self.stateFinished = False
        return method(self, *args, **kwargs)
    return wrapper


class SingleRun:
    def __init__(self, **kwargs):
        self.scriptPath = os.path.dirname(os.path.realpath(__file__))

        self.number = kwargs.get('number', '')
        self.sceneName = kwargs.get('scene')
        self.takeoff = kwargs.get('takeoff', False)
        
        self.config = json.load(open(os.path.join(utils_path, 'scenes', f'{self.sceneName}.json'), 'rb'))

        self.leadervelocityENU = np.array(self.config['velocityVectorENU'])
        self.formationTime = self.config["formationTime"]
        
        self.obstacleData = self.config.get('obstacleData', None)
        if self.obstacleData == None:
            self.nObstacle = 0
        else:
            self.nObstacle = len(self.obstacleData)
        self.config['nObstacle'] = self.nObstacle

        self.params = json.load(open(os.path.join(utils_path, 'params.json'), 'rb'))
        self.throttleTestOn = self.params['throttle_test']['on']
        self.throttleTestHeight = self.params['throttle_test']['height']
        self.throttleTestChangeTime = self.params['throttle_test']['change_time']
        self.throttleTestMinSim = self.params['throttle_test']['min_guess_sim']
        self.throttleTestMaxSim = self.params['throttle_test']['max_guess_sim']
        self.throttleTestMinReal = self.params['throttle_test']['min_guess_real']
        self.throttleTestMaxReal = self.params['throttle_test']['max_guess_real']
        self.throttleTestAccuracy = self.params['throttle_test']['accuracy']

        self.yawDegENU = self.params['yaw_deg_enu']
        self.yawRadENU = np.deg2rad(self.yawDegENU)
        self.safetyDistanceBetween = self.params['safety']['distance_between']
        self.safetyMaxHeight = self.params['safety']['max_height']
        self.safetyMinHeight = self.params['safety']['min_height']
        self.safetyXMin = self.params['safety']['x_min']
        self.safetyXMax = self.params['safety']['x_max']
        self.safetyYMin = self.params['safety']['y_min']
        self.safetyYMax = self.params['safety']['y_max']

        self.tStep = self.params['execution']['time_step']

        timeStr = kwargs.get('prefix', datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))

        self.folderName = os.path.join(self.scriptPath, '..', '..', '..', 'data', 'multi_uav_formation', 'SingleRun', timeStr)
        os.makedirs(self.folderName, exist_ok=True)

        cliOutputFile = open(os.path.join(self.folderName, f'output_{self.number}.txt'), "w")

        originalPrint = print
        def custom_print(*args, **kwargs):
            message = " ".join(map(str, args))
            originalPrint(message, **kwargs)
            cliOutputFile.write(message + "\n")
            cliOutputFile.flush()

        builtins.print = custom_print
        self.elapsedTime = self.tStep
        self.tUpperLimit = 100
        self.t = 0
        self.u = np.zeros((3,))
        self.data = []
        self.endControlFlag = 0

        self.state = State.INIT
        self.stateFinished = False
        self.taskStartTime = time.time()
        self.stateStartTime = time.time()
        self.taskTime = 0
        self.stateTime = 0
        
        self.dragCompensationENU = np.zeros(3)

        self.takeoffPointENU = np.array(self.config["takeoffPointENU"][self.number - 1])
        self.preparePointENU = np.array(self.config["preparePointENU"][self.number - 1])        

        rospy.init_node(f'single_run_{self.number}', anonymous=True)
        self.me = P230(name=f'uav{self.number}')
        self.communicator = Communicator(self)
        self.spinThread = threading.Thread(target=lambda: rospy.spin())
        self.spinThread.start()

        if not self.isLeader():
            from Visualizer import Visualizer
            self.visualizer = Visualizer()
            time.sleep(1)
            for ind, obstacle in self.obstacleData.items():
                self.visualizer.add_cylinder(obstacle['centerENU'], obstacle['radius'])
            self.visualizer.publish_once()

        self.timestamps = deque()
        self.hz = int(1 / self.tStep)

        if self.me.mode == 'sim':
            self.throttleTestMin = self.throttleTestMinSim
            self.throttleTestMax = self.throttleTestMaxSim
        elif self.me.mode == 'real':
            self.throttleTestMin = self.throttleTestMinReal
            self.throttleTestMax = self.throttleTestMaxReal
        else:
            raise Exception('Invalid mode')
        
        self.message = ''

    def addMessage(self, msg):
        self.message += msg + '\n'

    def isLeader(self):
        return self.number <= self.config['leaders']
    
    def getName(self):
        if self.isLeader():
            return f'Leader_{self.number}'
        else:
            return f'Follower_{self.number - self.config["leaders"]}'

    def number2Name(self, number, fullName=False):
        if number <= self.config["leaders"]:
            if fullName:
                return f'Leader_{number}'
            else:
                return f'l{number}'
        else:
            if fullName:
                return f'Follower_{number - self.config["leaders"]}'
            else:
                return f'f{number - self.config["leaders"]}'
            
    def name2Number(self, name):
        if name.startswith('l'):
            return eval(name[1:])
        elif name.startswith('f'):
            return eval(name[1:]) + self.config["leaders"]
        else:
            raise Exception(f'Invalid name: {name}')
    def isHead(self):
        return self.number == self.config['head']
    
    def othersAllAtState(self, state: State):
        return all([self.communicator.othersInfo[uav_number]['state'] == state.name for uav_number in range(1, self.config['number'] + 1) if uav_number != self.number])
    
    def othersAllAtFinishedState(self, state: State):
        return all([self.communicator.othersInfo[uav_number]['state'] == state.name and self.communicator.othersInfo[uav_number]['stateFinished'] == True for uav_number in range(1, self.config['number'] + 1) if uav_number != self.number])
    
    def headAtState(self, state: State):
        return self.communicator.othersInfo[self.config['head']]['state'] == state.name

    def getTimeNow(self):
        return time.time()

    @stepEntrance
    def toStepTakeoff(self):
        self.state = State.TAKEOFF
        self.me.setPositionControlMode()

    @stepEntrance
    def toStepAttitudeBiasTest(self):
        self.state = State.ATTITUDE_BIAS_TEST
        self.rollRecords = []
        self.pitchRecords = []

    @stepEntrance
    def toStepThrottleTest(self):
        self.state = State.THROTTLE_TEST
        self.throttleMin = self.throttleTestMin
        self.throttleMax = self.throttleTestMax
        self.throttle = (self.throttleMax + self.throttleMin) / 2.0
        self.changeTime = self.throttleTestChangeTime
        self.changeStep = self.throttleTestChangeTime
        self.throttleTestAdjustPosition = False
        self.lastVerVel = self.me.meVelocityENU[2]
        self.me.setAttitudeControlMode()

    @stepEntrance
    def toStepPrepare(self):
        self.state = State.PREPARE
        self.me.setPositionControlMode()

    @stepEntrance
    def toStepGuidance(self):
        self.state = State.GUIDANCE
        self.me.setAccelerationControlMode()

    @stepEntrance
    def toStepBack(self):
        self.state = State.BACK
        self.me.setPositionControlMode()

    @stepEntrance
    def toStepLand(self):
        self.state = State.LAND

    @stepEntrance
    def toStepEnd(self):
        self.state = State.END

    def isThisStateFinished(self):
        return self.stateFinished

    def finishState(self):
        self.stateFinished = True

    def stepInit(self):
        if not self.me.intoOffboardMode():
            return
        if self.me.meState.mode == 'OFFBOARD':
            self.finishState()
        if not self.takeoff:
            if not self.me.hasSetOrigin():
                self.me.setGpsOrigin()
                return
        if self.isThisStateFinished():
            if self.isHead():
                if self.othersAllAtFinishedState(State.INIT):
                    self.toStepTakeoff()
                else:
                    print(f'Waiting for others to enter state {State.INIT.name}')
            else:
                if self.headAtState(State.TAKEOFF):
                    self.toStepTakeoff()
                else:
                    print(f'Waiting for UAV #{self.config["head"]} to enter state {State.TAKEOFF.name}')

    def stepTakeoff(self):
        if self.isLeader():
            if self.number == 1: 
                if not self.me.isArmed():
                    self.me.arm()   
                self.me.positionENUControl(self.takeoffPointENU, self.yawRadENU)
            else:
                if self.communicator.othersInfo[self.number-1]['state'] == State.PREPARE.name and self.communicator.othersInfo[self.number-1]['stateFinished'] == True:
                    if not self.me.isArmed():
                        self.me.arm()
                    self.me.positionENUControl(self.takeoffPointENU, self.yawRadENU)
                else:
                    print(f'Waiting for leader {self.number-1} to enter state {State.PREPARE.name}')
        else:
            if self.communicator.othersInfo[self.config['leaders']]['state'] == State.PREPARE.name and self.communicator.othersInfo[self.config['leaders']]['stateFinished'] == True:
                if not self.me.isArmed():
                    self.me.arm()
                self.me.positionENUControl(self.takeoffPointENU, self.yawRadENU)
            else:
                print(f'Waiting for leaders to enter state {State.PREPARE.name}')

        if self.me.nearPositionENU(self.takeoffPointENU) and self.me.nearSpeed(0.0):
            self.finishState()
            if self.isLeader():
                self.toStepPrepare()
            else:
                self.toStepAttitudeBiasTest()

    def stepAttitudeBiasTest(self):
        self.me.hoverWithYaw(self.yawRadENU)
        if self.me.nearSpeed(0.0, tol=0.1):
            self.rollRecords.append(self.me.meRPYRadENU[0])
            self.pitchRecords.append(self.me.meRPYRadENU[1])
        print(f'{len(self.rollRecords) = }, {len(self.pitchRecords) = }')
        if self.stateTime >= 20.0 or (len(self.rollRecords) > 100 and len(self.pitchRecords) > 100):
            self.me.rollOffsetRad = np.mean(self.rollRecords)
            self.me.pitchOffsetRad = np.mean(self.pitchRecords)
            self.addMessage(f'Roll offset: {np.rad2deg(self.me.rollOffsetRad):.3f} deg, pitch offset: {np.rad2deg(self.me.pitchOffsetRad):.3f} deg')
            self.toStepThrottleTest()

    def stepThrottleTest(self):
        if self.stateTime >= 100.0:
            self.toStepLand()
            return
        
        if not self.safetyModule():
            self.toStepLand()
            return
        
        self.throttle = (self.throttleMin + self.throttleMax) / 2.0
        print(f'Between {self.throttleMin:.3f} and {self.throttleMax:.3f}: try {self.throttle:.3f}')
        print(f'Wait for {self.changeTime:.2f} to change, step is {self.changeStep:.2f}')
        print(f'Last vertical velocity @ {self.lastVerVel:.2f}')
        if self.stateTime >= self.changeTime:
            print(f'Now Vertical velocity end @ {self.me.meVelocityENU[2]:.2f}')
            if self.me.meVelocityENU[2] < self.lastVerVel:
                print(f'Hover throttle too low')
                self.throttleMin = self.throttle
            else:
                print(f'Hover throttle too high')
                self.throttleMax = self.throttle
            self.changeTime += self.changeStep
        self.lastVerVel = self.me.meVelocityENU[2]
                
        if self.throttleMax - self.throttleMin < self.throttleTestAccuracy:
            print(f'Throttle test result: between {self.throttleMin} and {self.throttleMax}')
            self.me.hoverThrottle = self.throttleMax
            self.toStepPrepare()
            return

        if self.throttleMax - self.throttleTestMin < 0.01:
            print(f'Throttle test failed: range to high')
            self.toStepLand()
            return

        if self.throttleTestMax - self.throttleMin < 0.01:
            print(f'Throttle test failed: range to low')
            self.toStepLand()
            return

        self.me.rpyENUThrustControl([0, 0, self.yawRadENU], self.throttle)

    def stepPrepare(self):
        print(f'Prepare to {arrayString(self.preparePointENU)}')
        self.me.velocityToPointENUControl(self.preparePointENU, self.yawRadENU)
        if self.me.nearPositionENU(self.preparePointENU) and self.me.nearSpeed(0.0):
            self.finishState()

        if self.isThisStateFinished():
            if self.isHead():
                if self.othersAllAtFinishedState(State.PREPARE):
                    self.toStepGuidance()
                else:
                    print(f'Waiting for others to enter state {State.PREPARE.name}')
            else:
                if self.headAtState(State.GUIDANCE):
                    self.toStepGuidance()
                else:
                    print(f'Waiting for UAV #{self.config["head"]} to enter state {State.GUIDANCE.name}')

    def stepGuidance(self):
        if self.isLeader():
            self.me.constantVelocityLineENUControl(self.leadervelocityENU, self.preparePointENU + self.stateTime * self.leadervelocityENU, self.yawRadENU)
        else:
            if not self.safetyModule():
                self.toStepBack()
                return
    
            self.u = np.array([0.0, -1.0, 0.0])
            print(f"guidanceCommandENU = {arrayString(self.u)}")

            thrust, self.cmdRPYRadENU = accENUYawENU2EulerENUThrust(
                accENU=self.u, 
                yawRadENU=self.yawRadENU, 
                hoverThrottle=self.me.hoverThrottle
            )
            self.me.rpyENUThrustControl(self.cmdRPYRadENU, thrust)

        self.log()
        if self.stateTime > self.formationTime:
            self.finishState()
            self.toStepBack()
    def name2positionENU(self, name):
        num = self.name2Number(name)
        position = self.communicator.othersInfo[num]['position']
        positionENU = np.array([position.x, position.y, position.z])
        return positionENU

    def name2velocityENU(self, name):
        num = self.name2Number(name)
        velocity = self.communicator.othersInfo[num]['velocity']
        velocityENU = np.array([velocity.x, velocity.y, velocity.z])
        return velocityENU

    def stepBack(self):
        self.me.velocityToPointENUControl(self.takeoffPointENU, self.yawRadENU)
        if self.me.nearPositionENU(self.takeoffPointENU):
            self.finishState()
            self.toStepLand()

    def stepLand(self):
        self.me.land()
        if self.me.belowHeight(height=0.2):
            self.finishState()
            self.toStepEnd()

    def controlStateMachine(self):
        if self.state == State.INIT:
            self.stepInit()
        elif self.state == State.TAKEOFF:
            self.stepTakeoff()
        elif self.state == State.THROTTLE_TEST:
            self.stepThrottleTest()
        elif self.state == State.PREPARE:
            self.stepPrepare()
        elif self.state == State.GUIDANCE:
            self.stepGuidance()
        elif self.state == State.BACK:
            self.stepBack()
        elif self.state == State.LAND:
            self.stepLand()
        elif self.state == State.END:
            exit(0)        
        elif self.state == State.ATTITUDE_BIAS_TEST:
            self.stepAttitudeBiasTest()

    def safetyModule(self):
        for uav_name, info in self.communicator.othersInfo.items():
            if self.me.nearPositionENU(point2Array(info['position']), tol=self.safetyDistanceBetween):
                print(f'Safety module: too close to {uav_name}, quit...')
                return False
        if self.me.aboveHeight(self.safetyMaxHeight):
            print(f'Safety module: too high, quit...')
            return False
        if self.me.belowHeight(self.safetyMinHeight):
            print(f'Safety module: too low, quit...')
            return False
        if self.me.mePositionENU[0] < self.safetyXMin or self.me.mePositionENU[0] > self.safetyXMax:
            print(f'Safety module: x ({self.me.mePositionENU[0]:.2f}) is out of range ({self.safetyXMin:.2f}, {self.safetyXMax:.2f}), quit...')
            return False
        if self.me.mePositionENU[1] < self.safetyYMin or self.me.mePositionENU[1] > self.safetyYMax:
            print(f'Safety module: y ({self.me.mePositionENU[1]:.2f}) is out of range ({self.safetyYMin:.2f}, {self.safetyYMax:.2f}), quit...')
            return False
        return True

    def print(self):
        os.system('clear')
        print('-' * 20)
        print(f'UAV #{self.me.name} / {self.number2Name(self.number, fullName=True)} : state {self.state.name}')
        print(f'Total time: {self.taskTime:.2f}, state time: {self.stateTime:.2f}, self.t: {self.t:.2f}')
        print((RED if self.hz < 0.8 / self.tStep else GREEN) + f"Current frequency: {self.hz} Hz" + RESET)
        print(f'Armed: {"YES" if self.me.isArmed() else "NO"}')
        self.me.printMe()
        print(self.message)

        for uav_number, info in self.communicator.othersInfo.items():
            state = info['state']
            stateFinished = info['stateFinished']
            position = info['position']
            velocity = info['velocity']
            if state is not None and stateFinished is not None and position is not None and velocity is not None:
                print(f'UAV #{uav_number}: state {state}, finished {stateFinished}, pos {pointString(position)}, vel {pointString(velocity)}')
            else:
                print(f'UAV #{uav_number}: No data')

    def updateFrequency(self):
        nowTime = time.time()
        
        self.timestamps.append(nowTime)
        
        while self.timestamps and self.timestamps[0] < nowTime - 1.0:
            self.timestamps.popleft()
        
        self.hz = len(self.timestamps)

    def run(self):
        self.me.sendHeartbeat()
        while self.state != State.END and not rospy.is_shutdown():
            tic = time.time()

            self.updateFrequency()

            self.taskTime = time.time() - self.taskStartTime
            self.stateTime = time.time() - self.stateStartTime

            self.print()

            self.controlStateMachine()
            self.me.sendHeartbeat()

            toc = time.time()
            self.elapsedTime = toc - tic
            if self.elapsedTime < self.tStep:
                time.sleep(self.tStep - self.elapsedTime)
                self.t += self.tStep
                self.elapsedTime = self.tStep
            else:
                self.t += self.elapsedTime
            print(f'Elapsed time: {self.elapsedTime:.4f}')

    def saveLog(self):
        self.fileName = os.path.join(self.folderName, f'data_{self.number}.pkl')
        with open(self.fileName, "wb") as file:
            pickle.dump({
                'sceneConfig': self.config, 
                'config': self.config, 
                'data': self.data, 
                'selfnumber': self.number,
                'params': self.params
                }, file)

        print(f"Data saved to {self.fileName}")

    def log(self):
        currentData = {}
        currentData['systemTime'] = copy.copy(datetime.datetime.now())
        currentData['t'] = copy.copy(self.t)
        currentData['u'] = copy.copy(self.u)
        currentData['mePositionENU'] = copy.copy(self.me.mePositionENU)
        currentData['mePositionNED'] = copy.copy(enu2ned(self.me.mePositionENU))
        currentData['meVelocity'] = copy.copy(self.me.meVelocityENU)
        currentData['meVelocityNorm'] = copy.copy(np.linalg.norm(self.me.meVelocityENU))
        currentData['meAccelerationENU'] = copy.copy(self.me.meAccelerationENU)    
        self.data.append(currentData)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--number', type=int, default=0, help='UAV number')
    parser.add_argument('--scene', type=str, default='scene1', help='Scene name')
    parser.add_argument('--takeoff', help='really takeoff or not', action='store_true')
    args = parser.parse_args()
    sr = SingleRun(**vars(args))
    sr.run()
    sr.saveLog()
    rospy.signal_shutdown('Shutting down')
    sr.spinThread.join()


if __name__ == '__main__':
    main()
