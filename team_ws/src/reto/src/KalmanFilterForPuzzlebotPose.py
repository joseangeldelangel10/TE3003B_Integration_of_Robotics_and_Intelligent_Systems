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
        rospy.Subscriber("/kalman_prediction_odom", Odometry, self.odom_callback)                
        rospy.Subscriber("/robot_pose_given_aruco_pose", Pose2D, self.visual_robot_pose_callback)
        #rospy.Subscriber("/kalman_prediction_odom",Odometry, self.kalman_odom_callback)

        
        #TODO - add the necessary publishers for the node
        #self.kalman_position_pub = rospy.Publisher("/kalman_odom", Odometry, queue_size=1)
        self.kalman_corrected_odom_pub = rospy.Publisher("/kalman_corrected_odom",Odometry, queue_size=1)
        self.kalman_position_message = Odometry()
        self.puzzlebot_6_times_6_covariance_matrix = np.zeros((6*6,), dtype=float)
        self.puzzlebot_height = 0.15

        self.relation_matrix_between_output_and_state = np.identity(3, dtype=float) # since the output of the systems equals its state (position) [C]
        self.state_uncertainty_matrix = np.ones((3,3), dtype=float)*0.8 # 5 meters in our context equals infinite        

        self.predicted_state = None
        self.predicted_state_for_kalman = None
        self.predicted_state_covariance = None
        self.predicted_state_covariance_for_kalman = None
        self.visual_sensor_reading = None

        self.rate_val = 2.0
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
        _, _, robot_yaw = nav_functions.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.predicted_state = np.array(
            [[msg.pose.pose.position.x],
             [msg.pose.pose.position.y],
             [robot_yaw]]
        )
        self.predicted_state_covariance = np.array(
            [[msg.pose.covariance[0], msg.pose.covariance[1], msg.pose.covariance[5] ],
             [msg.pose.covariance[6], msg.pose.covariance[7], msg.pose.covariance[11]],
             [msg.pose.covariance[30], msg.pose.covariance[31], msg.pose.covariance[35]]],
        )

    def kalman(self,xk_pred,Pk_pred,C,R,yk):
        #print("predicted state: \n {s}".format(s = xk_pred))
        #print("sensed state is: \n {s}".format(s = yk))
        #Correction
        Gk = Pk_pred@(C.T)@np.linalg.inv((C@Pk_pred@C.T)+R)        
        xk_corrected = xk_pred+ Gk@(yk-(C@xk_pred))  
        Pk_corrected = (np.identity(len(xk_pred), dtype=float)-(Gk@C))@Pk_pred
        Pk_corrected = np.absolute(Pk_corrected)
        return xk_corrected,Pk_corrected
    
    def fill_odometry_header(self):
        self.kalman_position_message.header.stamp = rospy.Time.now()
        self.kalman_position_message.header.frame_id = "map"
        self.kalman_position_message.child_frame_id = "base_link"
        # TODO self.puzzlebot_estimated_pose.pose.pose.position.z = <wheel rad>
    
    def create_kalman_position_message(self,corrected_state_mean, corrected_state_cov):
        
        self.fill_odometry_header()
        self.kalman_position_message.pose.pose.position.x = corrected_state_mean[0,0]
        self.kalman_position_message.pose.pose.position.y = corrected_state_mean[1,0]
        #self.kalman_position_message.pose.pose.position.z = self.puzzlebot_height/2.0

        self.puzzlebot_estimated_rot_quaternion = nav_functions.quaternion_from_euler(0.0, 0.0, corrected_state_mean[2,0])
        self.kalman_position_message.pose.pose.orientation.x = self.puzzlebot_estimated_rot_quaternion[0]
        self.kalman_position_message.pose.pose.orientation.y = self.puzzlebot_estimated_rot_quaternion[1]
        self.kalman_position_message.pose.pose.orientation.z = self.puzzlebot_estimated_rot_quaternion[2]
        self.kalman_position_message.pose.pose.orientation.w = self.puzzlebot_estimated_rot_quaternion[3]
                
        self.puzzlebot_6_times_6_covariance_matrix[0:2] = corrected_state_cov[0,0:2]
        self.puzzlebot_6_times_6_covariance_matrix[5] = corrected_state_cov[0,2]
        self.puzzlebot_6_times_6_covariance_matrix[6:8] = corrected_state_cov[1,0:2]
        self.puzzlebot_6_times_6_covariance_matrix[11] = corrected_state_cov[1,2]
        self.puzzlebot_6_times_6_covariance_matrix[30:32] = corrected_state_cov[2,0:2]
        self.puzzlebot_6_times_6_covariance_matrix[35] = corrected_state_cov[2,2]
        #flattened_6_times_6_covariance_matrix = self.puzzlebot_6_times_6_covariance_matrix.reshape((self.puzzlebot_6_times_6_covariance_matrix.shape[0]*self.puzzlebot_6_times_6_covariance_matrix.shape[1],))
        self.kalman_position_message.pose.covariance = self.puzzlebot_6_times_6_covariance_matrix.tolist()
        
        self.kalman_corrected_odom_pub.publish(self.kalman_position_message)
        #print("corrected_state_is: \n {s}".format(s = corrected_state_mean))
                
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
