#! /usr/bin/env python2

import rospy
import numpy as np
from ros_brain.srv import TriggerBrain, TriggerBrainResponse
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class Controller:

    def __init__(self):
        self.brain_srv = rospy.ServiceProxy('/trigger_brain', TriggerBrain)
        self.twist_publisher = rospy.Publisher(
            '/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.view_sub = rospy.Subscriber(
            '/view', Float64MultiArray, self.vision_callback, queue_size=1)
        self.twist = Twist()
        self.current_frame = None
        self.max_linear_speed = 0.3
        self.max_angular_speed = 1.0

    def vision_callback(self, msg):
        self.current_frame = msg.data
        resp = self.brain_srv(msg.data)
        # if resp.out_vec[0] > self.max_linear_speed or resp.out_vec[1] > self.max_angular_speed:
        #     rospy.logerr('Brain output exceeds maximum speed')
        #     if resp.out_vec[0] > self.max_linear_speed:
        #         linear_x = self.max_linear_speed
        #     else:
        #         linear_x = resp.out_vec[0]
        #     if resp.out_vec[1] > self.max_angular_speed:
        #         angular_z = self.max_angular_speed
        #     else:    
        #         angular_z = resp.out_vec[1]

        wz = (resp.out_vec[0] - 0.5)*2*(np.math.pi)
        self.twist.linear.x = self.max_linear_speed
        self.twist.angular.z = wz
        self.twist_publisher.publish(self.twist)

    def run(self):
        rospy.loginfo('turtle controller running')
        while not rospy.is_shutdown():  # and self.current_frame is not None:
            rospy.loginfo(self.twist)
            rospy.spin()


if __name__ == '__main__':

    rospy.init_node('turtle_controller')
    controller = Controller()
    controller.run()
