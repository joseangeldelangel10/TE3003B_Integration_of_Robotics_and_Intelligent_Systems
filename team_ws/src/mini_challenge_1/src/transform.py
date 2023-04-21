#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class Transform:
    
    def __init__(self, length =  0.192, radius = 0.057):
        rospy.init_node('transform')
        self.twist_sub = rospy.Subscriber('/cmd_vel', Twist, self.twist_callback)
        self.wl_pub = rospy.Publisher('/wl/qt', Float32, queue_size=1)
        self.wr_pub = rospy.Publisher('/wr/qt', Float32, queue_size=1)

        self.wr = Float32()
        self.wl = Float32()

        self.length = length
        self.radius = radius

        self.vel = None

        self.rate = rospy.Rate(10.0)    

    def twist_callback(self, msg):
        self.vel = msg.linear.x
        self.ang = msg.angular.z

    def main(self):
       while not rospy.is_shutdown():
            if self.vel is not None:
                
                self.wr = (2*self.vel + self.ang* self.length) / (2 * self.radius )
                self.wl = (-self.ang*self.length/self.radius) + self.wr
                
                self.wr_pub.publish(self.wr)
                self.wl_pub.publish(self.wl)

            self.rate.sleep()

if __name__ == '__main__':
    instance = Transform()
    instance.main()
