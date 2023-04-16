#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from mini_challenge_1.srv import *

class OdometryNode():
    # TODO fix this node so that it provides puzzlebot odometry using wheel speed based dead reckoning and
    # not Twist based odometry 

    def __init__(self, initial_x = 0.0, initial_y = 0.0, initial_theta = 0.0):
        rospy.init_node('odometry')
        self.initial_x, self.initial_y, self.initial_theta = (initial_x, initial_y, initial_theta)
        self.twist_sub = rospy.Subscriber('/cmd_vel', Twist, self.twist_callback)        
        self.puzzlebot_odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)        
        self.reset_puzzlebot_service_result = rospy.Service("reset_odometry", ResetOdometry, self.reset_odometry)
        self.puzzlebot_estimated_pose = Odometry()        
        self.fill_odometry_header()
        self.puzzlebot_estimated_pose.pose.pose.position.x, self.puzzlebot_estimated_pose.pose.pose.position.y = (self.initial_x, self.initial_y)        
        self.puzzlebot_estimated_rot = self.initial_theta
        self.puzzlebot_estimated_rot_quaternion = None
        self.puzzlebot_twist = None
        self.first_time_with_twist = True
        self.last_sampling_time = None
        self.rate = rospy.Rate(20.0)            
    
    def reset_odometry(self, req):
        self.puzzlebot_estimated_pose = Odometry()                
        self.fill_odometry_header()
        self.puzzlebot_estimated_pose.pose.pose.position.x, self.puzzlebot_estimated_pose.pose.pose.position.y = (self.initial_x, self.initial_y)        
        self.puzzlebot_estimated_rot = self.initial_theta
        self.puzzlebot_estimated_rot_quaternion = None
        self.puzzlebot_twist = None
        self.first_time_with_twist = True
        self.last_sampling_time = None
        rospy.loginfo("ODOMETRY RESET")
        return ResetOdometryResponse(True)

    def twist_callback(self, msg):
        """
        self.puzzlebot_vel_on_base_frame_x = msg.linear.x
        self.puzzlebot_vel_on_base_frame_y = msg.linear.y
        self.puzzlebot_vel_on_base_frame_th = msg.angular.z
        """
        self.puzzlebot_twist = msg

    def fill_odometry_header(self):
        self.puzzlebot_estimated_pose.header.stamp = rospy.Time.now()
        self.puzzlebot_estimated_pose.header.frame_id = "world"
        self.puzzlebot_estimated_pose.child_frame_id = "base_link"
        # TODO self.puzzlebot_estimated_pose.pose.pose.position.z = <wheel rad>        

    def main(self):
        while not rospy.is_shutdown():
            if self.puzzlebot_twist is not None:
                if self.first_time_with_twist:
                    self.first_time_with_twist = False
                    self.last_sampling_time = rospy.get_time()
                    rospy.loginfo("Odometry initialized")
                else:
                    current_time = rospy.get_time()
                    delta_t = (current_time - self.last_sampling_time)
                    self.fill_odometry_header()
                    self.puzzlebot_estimated_pose.pose.pose.position.x += self.puzzlebot_twist.linear.x*(np.cos(self.puzzlebot_estimated_rot))*delta_t
                    self.puzzlebot_estimated_pose.pose.pose.position.y += self.puzzlebot_twist.linear.x*(np.sin(self.puzzlebot_estimated_rot))*delta_t
                    self.puzzlebot_estimated_rot += self.puzzlebot_twist.angular.z*delta_t
                    self.puzzlebot_estimated_rot_quaternion = quaternion_from_euler(0.0, 0.0, self.puzzlebot_estimated_rot)
                    self.puzzlebot_estimated_pose.pose.pose.orientation.x = self.puzzlebot_estimated_rot_quaternion[0]
                    self.puzzlebot_estimated_pose.pose.pose.orientation.y = self.puzzlebot_estimated_rot_quaternion[1]
                    self.puzzlebot_estimated_pose.pose.pose.orientation.z = self.puzzlebot_estimated_rot_quaternion[2]
                    self.puzzlebot_estimated_pose.pose.pose.orientation.w = self.puzzlebot_estimated_rot_quaternion[3]
                    self.last_sampling_time = current_time
                    self.puzzlebot_estimated_pose.twist.twist = self.puzzlebot_twist
            self.puzzlebot_odom_pub.publish(self.puzzlebot_estimated_pose)
            self.rate.sleep()

if __name__ == '__main__':
    instance = OdometryNode()
    instance.main()