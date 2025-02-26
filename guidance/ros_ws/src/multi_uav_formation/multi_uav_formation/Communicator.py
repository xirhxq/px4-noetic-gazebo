import rospy
from std_msgs.msg import UInt8, Bool
from geometry_msgs.msg import PoseStamped, TwistStamped
import threading

from State import State

class Communicator:
    def __init__(self, singleRunInstance):
        self.singleRunInstance = singleRunInstance
        self.uavNumber = singleRunInstance.number
        self.uavRange = range(1, singleRunInstance.config['number'] + 1)
        self.mePublishers = {}
        self.meSubscribers = {}
        self.othersInfo = {i: {
            'state': None, 
            'stateFinished': None, 
            'position': None,
            'velocity': None
            } for i in self.uavRange if i != self.uavNumber}

        self.createPubSub('state', UInt8)
        self.createPubSub('stateFinished', Bool)
        self.createPubSub('pose', PoseStamped)
        self.createPubSub('velocity_local', TwistStamped)

        self.publishFrequency = 10
        self.publishTimer = rospy.Timer(rospy.Duration(1.0 / self.publishFrequency), self.publishInfo)
        self.spinThread = threading.Thread(target=rospy.spin)
        self.spinThread.start()

    def createPubSub(self, topic_suffix, msg_type):
        topic_name = f'/uav_{self.uavNumber}/{topic_suffix}'
        self.mePublishers[topic_suffix] = rospy.Publisher(topic_name, msg_type, queue_size=10)

        for i in self.uavRange:
            if i != self.uavNumber:
                sub_topic_name = f'/uav_{i}/{topic_suffix}'
                self.meSubscribers[f'{i}_{topic_suffix}'] = rospy.Subscriber(
                    sub_topic_name, msg_type, lambda msg, i=i: self.callback(msg, i, topic_suffix))

    def publishInfo(self, event):
        state_msg = UInt8()
        state_msg.data = self.singleRunInstance.state.value
        self.mePublishers['state'].publish(state_msg)

        state_finished_msg = Bool()
        state_finished_msg.data = self.singleRunInstance.stateFinished
        self.mePublishers['stateFinished'].publish(state_finished_msg)

        position_msg = PoseStamped()
        position_msg.header.frame_id = 'map'
        position_msg.pose.position.x = self.singleRunInstance.me.mePositionENU[0]
        position_msg.pose.position.y = self.singleRunInstance.me.mePositionENU[1]
        position_msg.pose.position.z = self.singleRunInstance.me.mePositionENU[2]
        self.mePublishers['pose'].publish(position_msg)

        velocity_msg = TwistStamped()
        velocity_msg.twist.linear.x = self.singleRunInstance.me.meVelocityENU[0]
        velocity_msg.twist.linear.y = self.singleRunInstance.me.meVelocityENU[1]
        velocity_msg.twist.linear.z = self.singleRunInstance.me.meVelocityENU[2]
        self.mePublishers['velocity_local'].publish(velocity_msg)

    def callback(self, msg, uavNumber, topic_suffix):
        if topic_suffix == 'state':
            self.othersInfo[uavNumber]['state'] = State(msg.data).name
        elif topic_suffix == 'stateFinished':
            self.othersInfo[uavNumber]['stateFinished'] = msg.data
        elif topic_suffix == 'pose':
            position = PoseStamped()
            position.pose.position.x = msg.pose.position.x
            position.pose.position.y = msg.pose.position.y
            position.pose.position.z = msg.pose.position.z
            self.othersInfo[uavNumber]['position'] = position.pose.position
        elif topic_suffix == 'velocity_local':
            velocity = TwistStamped()
            velocity.twist.linear.x = msg.twist.linear.x
            velocity.twist.linear.y = msg.twist.linear.y
            velocity.twist.linear.z = msg.twist.linear.z
            self.othersInfo[uavNumber]['velocity'] = velocity.twist.linear

    def stop(self):
        rospy.signal_shutdown('Communicator stopped')
        self.spinThread.join()
