#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Float32
from mini_challenge_1.srv import *

class Solver:
    
    def __init__(self, initial_x = 0.0, initial_y = 0.0, initial_theta = 0.0, inicial_left_wheel_angle = 0.0, inicial_right_wheel_angle = 0.0):
        rospy.init_node('solver')
        self.twist_sub = rospy.Subscriber('/puzzlebot_vel_on_base_frame_pub', Twist, self.twist_callback)

        self.puzzlebot_pose_pub = rospy.Publisher('/pose', Pose2D, queue_size=1)
        self.puzzlebot_pose_pub = rospy.Publisher('/pose', Pose2D, queue_size=1)        
        self.puzzlebot_pose = Pose2D()
        self.reset_puzzlebot_service_result = rospy.Service("reset_puzzlebot_sim", ResetPuzzlebotSim, self.reset_puzzlebot_sim)
        self.puzzlebot_pose.x, self.puzzlebot_pose.y, self.puzzlebot_pose.theta = (initial_x, initial_y, initial_theta)
        self.puzzlebot_vel_on_base_frame_x = None
        self.puzzlebot_vel_on_base_frame_y = None
        self.puzzlebot_vel_on_base_frame_th = None
        self.first_time_with_twist = True

        self.wl_sub = rospy.Subscriber('/wl', Float32, self.wl_callback)
        self.wr_sub = rospy.Subscriber('/wr', Float32, self.wr_callback)
        self.left_wheel_angle_pub = rospy.Publisher('/left_wheel_angle', Float32, queue_size=1)
        self.right_wheel_angle_pub = rospy.Publisher('/right_wheel_angle', Float32, queue_size=1)
        self.left_wheel_angle, self.right_wheel_angle = inicial_left_wheel_angle, inicial_right_wheel_angle
        self.wr = None
        self.wl = None

        self.last_sampling_time = None
        self.rate = rospy.Rate(10.0)    

    def reset_puzzlebot_sim(self, req):
        self.puzzlebot_pose.x = 0.0
        self.puzzlebot_pose.y = 0.0
        self.puzzlebot_pose.theta = 0.0
        return ResetPuzzlebotSimResponse(True)

    def twist_callback(self, msg):
        self.puzzlebot_vel_on_base_frame_x = msg.linear.x
        self.puzzlebot_vel_on_base_frame_y = msg.linear.y
        self.puzzlebot_vel_on_base_frame_th = msg.angular.z

    def wl_callback(self, msg):
        self.wl = msg.data
    
    def wr_callback(self, msg):
        self.wr = msg.data

    def main(self):
        while not rospy.is_shutdown():
            if self.puzzlebot_vel_on_base_frame_x is not None and self.wl is not None and self.wr is not None:
                if self.first_time_with_twist:
                    self.first_time_with_twist = False
                    self.last_sampling_time = rospy.get_time()
                    print("fist time vel passed")
                else:
                    current_time = rospy.get_time()
                    delta_t = (current_time - self.last_sampling_time)
                    self.puzzlebot_pose.x += self.puzzlebot_vel_on_base_frame_x*delta_t
                    self.puzzlebot_pose.y += self.puzzlebot_vel_on_base_frame_y*delta_t
                    self.puzzlebot_pose.theta += self.puzzlebot_vel_on_base_frame_th*delta_t
                    self.left_wheel_angle += self.wl*delta_t
                    self.right_wheel_angle += self.wr*delta_t
                    self.last_sampling_time = current_time


            self.puzzlebot_pose_pub.publish(self.puzzlebot_pose)
            self.left_wheel_angle_pub.publish(self.left_wheel_angle)
            self.right_wheel_angle_pub.publish(self.right_wheel_angle)
            self.rate.sleep()

if __name__ == '__main__':
    instance = Solver()
    instance.main()