#! /usr/bin/env python2

import rospy
from ros_brain.srv import TriggerBrain, TriggerBrainResponse
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
# from webcam.msg import View

class Controller:

    def __init__(self):
        self.brain_srv = rospy.ServiceProxy('/trigger_brain',TriggerBrain)
        self.twist_publisher = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=1)
        self.view_sub = rospy.Subscriber('/view',Float64MultiArray,self.vision_callback,queue_size=1)
        self.twist= Twist()
        self.current_frame = None

    def vision_callback(self, msg):
        self.current_frame=msg.data
        resp = self.brain_srv(msg.data)
        self.twist.linear.x = resp.out_vec[0]
        self.twist.angular.z =resp.out_vec[1]
        self.twist_publisher.publish(self.twist)

    def run(self):
        rospy.loginfo('turtle controller running')

        while not rospy.is_shutdown(): #and self.current_frame is not None:
            rospy.spin()








if __name__== '__main__':

    rospy.init_node('turtle_controller')
    controller = Controller()
    controller.run()