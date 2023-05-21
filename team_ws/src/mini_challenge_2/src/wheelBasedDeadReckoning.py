#!/usr/bin/env python

import rospy
import random as rm
import numpy as np
from std_msgs.msg import Float32, Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from mini_challenge_1.srv import *
from mini_challenge_2.srv import *
import rospkg
import os
import json

class OdometryNode():
    # TODO fix this node so that it provides puzzlebot odometry using wheel speed based dead reckoning and
    # not Twist based dead reckoning
    
    def __init__(self, initial_x = 0.0, initial_y = 0.0, initial_theta = 0.0):
        
        # ______________ init node publishers, subscribers and services ______________
        rospy.init_node('odometry')        
        self.wr_sub = rospy.Subscriber('/wr', Float32, self.puzzlebot_wr_callback)
        self.wl_sub = rospy.Subscriber('/wl', Float32, self.puzzlebot_wl_callback)        
        self.puzzlebot_odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.puzzlebot_simple_odom_pub = rospy.Publisher('/simple_odom', Pose2D, queue_size=1)        
        self.reset_puzzlebot_service_result = rospy.Service("reset_odometry", ResetOdometry, self.reset_odometry)
        
        self.get_covariance_mat_server = rospy.Service("get_puzzlebot_covariance_mat_on_sim", GetPuzzlebotCovarianceMatOnSim, self.get_covariance_mat)        
        
        # ______________ fill in publisher messages ______________
        self.initial_x, self.initial_y, self.initial_theta = (initial_x, initial_y, initial_theta)        
        self.puzzlebot_estimated_pose = Odometry()        
        self.puzzlebot_simple_odom_msg = Pose2D()
        self.fill_odometry_header()        
        self.puzzlebot_estimated_pose.pose.pose.position.x, self.puzzlebot_estimated_pose.pose.pose.position.y = (self.initial_x, self.initial_y)        
        self.puzzlebot_estimated_rot = self.initial_theta        
        
        # ______________ init nav variables ______________
        self.puzzlebot_estimated_rot_quaternion = None
        self.puzzlebot_twist = Twist()
        self.first_time_with_twist = True
        self.last_sampling_time = None        
        self.puzzlebot_wheel_rad = 0.057 # radius
        self.puzzlebot_wheel_to_wheel_dist = 0.192 # length
        self.puzzlebot_wr = None
        self.puzzlebot_wl = None
        self.v = 0.0
        self.w = 0.0
        
        # ______________ init covariance variables ______________
        self.covariance_matrix = np.zeros((3,3))
        self.puzzlebot_model_jacobian = None        
        self.nabla_w_k = None
        self.sigma_delta_k = None
        self.q_k = None
        rp = rospkg.RosPack()
        this_pkg_path = rp.get_path('mini_challenge_2')
        cov_constants_file_name = os.path.join(this_pkg_path, "src", "constants", "covariance_prop_constants.json")
        cov_constants_file = open(cov_constants_file_name)
        cov_constants = json.load(cov_constants_file)        
        self.covar_kr = cov_constants["kr"]
        self.covar_kl = cov_constants["kl"]

        # ______________ covariance mat service variables ______________
        self.get_covariance_mat_result = Float64MultiArray()
        self.covariance_mat_height_layout = MultiArrayDimension()
        self.covariance_mat_width_layout = MultiArrayDimension()
        self.covariance_mat_height_layout.label = "height"
        self.covariance_mat_height_layout.size = 3
        self.covariance_mat_height_layout.stride = 3*3
        self.covariance_mat_width_layout.label = "width"
        self.covariance_mat_width_layout.size = 3
        self.covariance_mat_width_layout.stride = 3

        # ______________ init rate ______________
        self.rate = rospy.Rate(20.0)            
    
    def get_covariance_mat(self, req):
        self.get_covariance_mat_result.layout.dim = [self.covariance_mat_height_layout, self.covariance_mat_width_layout]        
        self.get_covariance_mat_result.layout.data_offset = 0
        self.get_covariance_mat_result.data = self.covariance_matrix.reshape((3*3,)) 
        return GetPuzzlebotCovarianceMatOnSimResponse(self.get_covariance_mat_result)

    def reset_odometry(self, req):
        self.puzzlebot_estimated_pose = Odometry()                
        self.fill_odometry_header()
        self.puzzlebot_estimated_pose.pose.pose.position.x, self.puzzlebot_estimated_pose.pose.pose.position.y = (self.initial_x, self.initial_y)        
        self.puzzlebot_estimated_rot = self.initial_theta
        self.puzzlebot_estimated_rot_quaternion = None
        self.puzzlebot_twist = Twist()
        self.first_time_with_twist = True
        self.last_sampling_time = None
        self.puzzlebot_wr = None
        self.puzzlebot_wl = None
        self.v = 0.0
        self.w = 0.0                
        self.covariance_matrix = np.zeros((3,3))
        self.puzzlebot_model_jacobian = None        
        self.nabla_w_k = None
        self.sigma_delta_k = None
        self.q_k = None
        rp = rospkg.RosPack()
        this_pkg_path = rp.get_path('mini_challenge_2')
        cov_constants_file_name = os.path.join(this_pkg_path, "src", "constants", "covariance_prop_constants.json")
        cov_constants_file = open(cov_constants_file_name)
        cov_constants = json.load(cov_constants_file)        
        self.covar_kr = cov_constants["kr"]
        self.covar_kl = cov_constants["kl"]
        rospy.loginfo("ODOMETRY RESET")        
        return ResetOdometryResponse(True)

    def puzzlebot_wr_callback(self, msg):
        self.puzzlebot_wr = msg.data        

    def puzzlebot_wl_callback(self, msg):
        self.puzzlebot_wl = msg.data         

    def fill_odometry_header(self):
        self.puzzlebot_estimated_pose.header.stamp = rospy.Time.now()
        self.puzzlebot_estimated_pose.header.frame_id = "map"
        self.puzzlebot_estimated_pose.child_frame_id = "base_link"
        # TODO self.puzzlebot_estimated_pose.pose.pose.position.z = <wheel rad>        

    def main(self):
        while not rospy.is_shutdown():
            if self.puzzlebot_wr is not None and self.puzzlebot_wl is not None:
                if self.first_time_with_twist:
                    self.first_time_with_twist = False
                    self.last_sampling_time = rospy.get_time()                    
                    rospy.loginfo("Odometry initialized")
                else:
                    current_time = rospy.get_time()
                    delta_t = (current_time - self.last_sampling_time)
                    self.fill_odometry_header()
                    
                     # _________ wheels' odometry ______________
                    #publicadores nuevos? v y w
                    self.v = (self.puzzlebot_wheel_rad/2)*self.puzzlebot_wr + (self.puzzlebot_wheel_rad/2)*self.puzzlebot_wl
                    self.w = (self.puzzlebot_wheel_rad/self.puzzlebot_wheel_to_wheel_dist)*self.puzzlebot_wr - (self.puzzlebot_wheel_rad/self.puzzlebot_wheel_to_wheel_dist)*self.puzzlebot_wl
                    # _________ end of wheels' odometry ______________

                    # _________ filling puzzlebot covariance data ______________                    
                    self.puzzlebot_model_jacobian = np.array(
                        [[1.0, 0.0, -delta_t* self.v* np.sin(self.puzzlebot_estimated_rot) ],
                         [0.0, 1.0, delta_t* self.v* np.cos(self.puzzlebot_estimated_rot) ],
                         [0.0, 0.0, 1.0]] 
                    )                    
                    multiplier = (1.0/2.0)*self.puzzlebot_wheel_rad*delta_t
                    self.nabla_w_k = multiplier*np.array(
                        [[np.cos(self.puzzlebot_estimated_rot), np.cos(self.puzzlebot_estimated_rot)],
                         [np.sin(self.puzzlebot_estimated_rot), np.sin(self.puzzlebot_estimated_rot)],
                         [(2/self.puzzlebot_wheel_to_wheel_dist), -(2/self.puzzlebot_wheel_to_wheel_dist)]]
                    )
                    self.sigma_delta_k = np.array(
                        [[self.covar_kr*abs(self.puzzlebot_wr), 0.0],
                         [0.0, self.covar_kl*abs(self.puzzlebot_wl)]]
                    )                    
                    self.q_k = np.matmul( np.matmul( self.nabla_w_k , self.sigma_delta_k ), self.nabla_w_k.T )                    
                    self.covariance_matrix = np.matmul( np.matmul(self.puzzlebot_model_jacobian,self.covariance_matrix), (self.puzzlebot_model_jacobian.T)) + self.q_k
                    self.puzzlebot_estimated_pose.pose.covariance = (
                        self.covariance_matrix[0,0:2].tolist() + [0.0] * 3 + [self.covariance_matrix[0,2]] +
                        self.covariance_matrix[1,0:2].tolist() + [0.0] * 3 + [self.covariance_matrix[1,2]] +
                        [0.0] * 6 +
                        [0.0] * 6 +
                        [0.0] * 6 +
                        self.covariance_matrix[2,0:2].tolist() + [0.0] * 3 + [self.covariance_matrix[2,2]]                                         
                    )
                    # _________ end of filling puzzlebot covariance data ______________
                    
                    
                    # _________ wheels' odometry ______________
                    #publicadores nuevos? v y w
                    self.v = (self.puzzlebot_wheel_rad/2)*self.puzzlebot_wr + (self.puzzlebot_wheel_rad/2)*self.puzzlebot_wl
                    self.w = (self.puzzlebot_wheel_rad/self.puzzlebot_wheel_to_wheel_dist)*self.puzzlebot_wr - (self.puzzlebot_wheel_rad/self.puzzlebot_wheel_to_wheel_dist)*self.puzzlebot_wl
                    # _________ end of wheels' odometry ______________


                    # _________ filling puzzlebot state data ______________
                    self.puzzlebot_estimated_pose.pose.pose.position.x += self.v*(np.cos(self.puzzlebot_estimated_rot))*delta_t
                    self.puzzlebot_estimated_pose.pose.pose.position.y += self.v*(np.sin(self.puzzlebot_estimated_rot))*delta_t
                    self.puzzlebot_estimated_rot += self.w*delta_t
                    self.puzzlebot_estimated_rot_quaternion = quaternion_from_euler(0.0, 0.0, self.puzzlebot_estimated_rot)
                    self.puzzlebot_estimated_pose.pose.pose.orientation.x = self.puzzlebot_estimated_rot_quaternion[0]
                    self.puzzlebot_estimated_pose.pose.pose.orientation.y = self.puzzlebot_estimated_rot_quaternion[1]
                    self.puzzlebot_estimated_pose.pose.pose.orientation.z = self.puzzlebot_estimated_rot_quaternion[2]
                    self.puzzlebot_estimated_pose.pose.pose.orientation.w = self.puzzlebot_estimated_rot_quaternion[3]
                    self.puzzlebot_twist.linear.x = self.v
                    self.puzzlebot_twist.angular.z = self.w
                    self.puzzlebot_estimated_pose.twist.twist = self.puzzlebot_twist
                    # _________ end of filling puzzlebot pose data ______________

                    
                    # _________ filling simple odom message _________
                    self.puzzlebot_simple_odom_msg.x = self.puzzlebot_estimated_pose.pose.pose.position.x
                    self.puzzlebot_simple_odom_msg.y = self.puzzlebot_estimated_pose.pose.pose.position.y
                    self.puzzlebot_simple_odom_msg.theta = self.puzzlebot_estimated_rot%(2.0*np.pi)
                    # _________ end of filling simple odom message _________

                    self.last_sampling_time = current_time
            self.puzzlebot_simple_odom_pub.publish(self.puzzlebot_simple_odom_msg)                    
            self.puzzlebot_odom_pub.publish(self.puzzlebot_estimated_pose)
            self.rate.sleep()

if __name__ == '__main__':
    instance = OdometryNode()
    instance.main()
