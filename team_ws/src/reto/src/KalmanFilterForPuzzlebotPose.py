#!/usr/bin/env python3

"""Made by:
    Leonardo Javier Nava
        navaleonardo40@gmail.com
    Jose Angel del Angel Dominguez
        joseangeldelangel10@gmail.com

Code description:
    Ros node that implements the Kalman filter for manchester robotics puzlebot 
    considering the odometry info as a priori information and computer vision as
    posteriori info.
Notes:
"""

import math
import rospy
import numpy as np
import nav_functions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D


class KalmanFilterForPuzzlebotPose():
    def __init__(self):
        
        rospy.init_node("kalman_filter_for_puzzlebot_pose")
        rospy.Subscriber("/odom", Odometry, self.odom_callback)                
        rospy.Subscriber("/robot_pose_given_aruco_pose", Pose2D, self.visual_robot_pose_callback)
        
        #TODO - add the necessary publishers for the node
        self.kalman_position_pub = rospy.Publisher("/kalman_odom", Odometry, queue_size=1)
        self.kalman_position_message = Odometry()
        self.puzzlebot_6_times_6_covariance_matrix = np.zeros((6,6))
        self.puzzlebot_height = 15.0

        self.relation_matrix_between_output_and_state = np.identity(3) # since the output of the systems equals its state (position) [C]
        self.state_uncertainty_matrix = np.ones((3,3))*5 # 5 meters in our context equals infinite        

        self.predicted_state = None
        self.predicted_state_covariance = None
        self.visual_sensor_reading = None

        self.rate_val = 20.0
        self.rate = rospy.Rate(self.rate_val)    

    def odom_callback(self, data):
        self.odom_msg_to_state_vector(data)
    
    def visual_robot_pose_callback(self, msg):
        self.visual_sensor_reading = np.array(
            [[msg.x],
             [msg.y],
             [msg.theta]]
        )

    def odom_msg_to_state_vector(self, msg):
        robot_yaw = nav_functions.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.predicted_state = np.array(
            [[msg.pose.pose.position.x],
             [msg.pose.pose.position.y],
             [robot_yaw]]
        )
        self.predicted_state_covariance = np.array(
            [[self.pose.covariance[0], self.pose.covariance[1], self.pose.covariance[5] ],
             [self.pose.covariance[6], self.pose.covariance[7], self.pose.covariance[11]],
             [self.pose.covariance[30], self.pose.covariance[31], self.pose.covariance[35]]]
        )

    def kalman(self,xk_pred,Pk_pred,C,R,yk):
        #Correction
        Gk = Pk_pred@(C.T)@np.linalg.inv((C@Pk_pred@C.T)+R)
        xk_corrected = xk_pred+ Gk@(yk-(C@xk_pred))  
        Pk_corrected = (np.identity(len(xk_pred))-(Gk@C))@Pk_pred
        return xk_corrected,Pk_corrected
    
    def create_kalman_position_message(self,corrected_state_mean, corrected_state_cov):
        
        self.kalman_position_message.pose.pose.position.x = corrected_state_mean[0]
        self.kalman_position_message.pose.pose.position.y = corrected_state_mean[1]
        self.kalman_position_message.pose.pose.position.z = self.puzzlebot_height/2.0
                
        self.puzzlebot_6_times_6_covariance_matrix[0,:2] = corrected_state_cov[0,:2]
        self.puzzlebot_6_times_6_covariance_matrix[0,5] = corrected_state_cov[0,2]
        self.puzzlebot_6_times_6_covariance_matrix[5,:2] = corrected_state_cov[2,:2]
        self.puzzlebot_6_times_6_covariance_matrix[5,5] = corrected_state_cov[2,2]
        flattened_6_times_6_covariance_matrix = self.puzzlebot_6_times_6_covariance_matrix.reshape((self.puzzlebot_6_times_6_covariance_matrix.shape[0]*self.puzzlebot_6_times_6_covariance_matrix.shape[1],))
        self.kalman_position_message.pose.covariance = flattened_6_times_6_covariance_matrix.tolist()
                
        self.kalman_position_pub.publish(self.kalman_position_message)
                
    def main(self):        
        while not rospy.is_shutdown():
            if self.predicted_state is not None and self.predicted_state_covariance is not None and self.visual_sensor_reading is not None:
                corrected_state_mean, corrected_state_cov = self.kalman(self.predicted_state,
                                                                        self.predicted_state_covariance,
                                                                        self.relation_matrix_between_output_and_state,
                                                                        self.state_uncertainty_matrix,
                                                                        self.visual_sensor_reading)
                
                self.create_kalman_position_message(corrected_state_mean, corrected_state_cov)
                
            self.rate.sleep()          

if __name__ == "__main__":
    kalman_filer = KalmanFilterForPuzzlebotPose()
    kalman_filer.main()
