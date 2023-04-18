#!/usr/bin/env python
import rospy
import pandas as pd
import rospkg
import os
from geometry_msgs.msg import Twist, Pose2D, Pose
from nav_msgs.msg import Odometry
from mini_challenge_1.srv import *
import numpy as np
import json
import random

class CovarianceConstantsAutoTuner():
    def __init__(self):        
        rospy.init_node('auto_tune_covariance_constants')
        self.epochs = 20
        self.experiments_linear_vel_ang_vel_and_exec_time = [
            (0.2, 0.0, 2.0),
            (0.2, 0.0, 4.0),
            (0.2, 0.0, 6.0),
            (0.0, np.pi/5, 1.0),
            (0.0, np.pi/5, 2.0),
            (0.0, np.pi/5, 3.0),
            ]        
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.cmd_vel_msg = Twist()
        self.previous_kr = None
        self.previous_kl = None
        self.current_kr = None
        self.current_kl = None
        self.previous_err = None
        self.current_err = 0.0        
        self.learning_rate = 0.01
        
        self.first_time = True
        self.inital_time = None
        self.experiment_index = 0

        self.rate = rospy.Rate(20.0)

    def publish_vel(self, linear_vel, angular_vel):
        self.cmd_vel_msg.linear.x = linear_vel
        self.cmd_vel_msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(self.cmd_vel_msg)  

    def get_covariance_constants(self):
        rp = rospkg.RosPack()
        this_pkg_path = rp.get_path('mini_challenge_2')
        cov_constants_file_name = os.path.join(this_pkg_path, "src", "constants", "covariance_prop_constants.json")
        cov_constants_file = open(cov_constants_file_name, "r")
        cov_constants = json.load(cov_constants_file)        
        return (cov_constants["kr"], cov_constants["kl"])
    
    def rewrite_covariance_constants(self, new_kr, new_kl):
        rp = rospkg.RosPack()
        this_pkg_path = rp.get_path('mini_challenge_2')
        cov_constants_file_name = os.path.join(this_pkg_path, "src", "constants", "covariance_prop_constants.json")
        cov_constants_file = open(cov_constants_file_name, "r")
        cov_constants = json.load(cov_constants_file)    
        cov_constants["kr"] = new_kr
        cov_constants["kl"] = new_kl
        cov_constants_file_w = open(cov_constants_file_name, "w")
        json.dump(cov_constants, cov_constants_file_w)                

    def reset_puzzlebot_sim(self):
        rospy.wait_for_service('reset_puzzlebot_sim')
        try:
            reset_sim = rospy.ServiceProxy('reset_puzzlebot_sim', ResetPuzzlebotSim)
            response = reset_sim()
            return response.success
        except rospy.ServiceException:
            print("reset sim service call failed")  

    def reset_odometry(self):
        rospy.wait_for_service('reset_odometry')
        try:
            reset_odometry = rospy.ServiceProxy('reset_odometry', ResetOdometry)
            response = reset_odometry()
            return response.success
        except rospy.ServiceException:
            print("reset odom service call failed")        

    def main(self):
        for i in range(self.epochs):
            while not rospy.is_shutdown():
                linear_vel, ang_vel, exec_time = self.experiments_linear_vel_ang_vel_and_exec_time[self.experiment_index]                                        
                if self.first_time:
                    self.inital_time = rospy.get_time()
                else:
                    if (rospy.get_time() - self.inital_time) < exec_time:
                        self.publish_vel(linear_vel, ang_vel)
                    else:
                        self.publish_vel(0.0, 0.0)
                        if self.experiment_index >= (len(self.experiments_linear_vel_ang_vel_and_exec_time) -1):
                            experiment_cov_matrix = None # TODO change this when service is done 
                            sim_cov_matrix = None # TODO change this when service is done
                            diff_mat = experiment_cov_matrix - sim_cov_matrix
                            self.current_err +=  (diff_mat**2).sum()

                            self.current_kr, self.current_kl = self.get_covariance_constants()
                            if self.previous_err is None:                                                        
                                new_kr = self.current_kr + random.uniform(-0.2, 0.2)
                                new_kl = self.current_kl + random.uniform(-0.2, 0.2)
                                self.rewrite_covariance_constants(new_kr, new_kl)
                            else:
                                kr_error_derivative = (self.current_err - self.previous_err)/(self.current_kr - self.previous_kr)
                                kl_error_derivative = (self.current_err - self.previous_err)/(self.current_kl - self.previous_kl)
                                new_kr = self.current_kr - self.learning_rate*kr_error_derivative
                                new_kl = self.current_kl - self.learning_rate*kl_error_derivative
                                self.rewrite_covariance_constants(new_kr, new_kl)
                            self.previous_err = self.current_err
                            self.previous_kr, self.previous_kl = (self.current_kr, self.current_kl)
                            self.current_err = 0.0
                            self.first_time = True
                            self.inital_time = None
                            self.experiment_index = 0
                            break
                        else:
                            experiment_cov_matrix = None # TODO change this when service is done 
                            sim_cov_matrix = None # TODO change this when service is done
                            diff_mat = experiment_cov_matrix - sim_cov_matrix
                            self.current_err +=  (diff_mat**2).sum()
                            self.experiment_index += 1
                        #if self.previous_kr is None and self.previous_kl is None:
                        # TODO implement autotuning logic
                self.rate.sleep()        
        

if __name__ == '__main__':    
    auto_tuner = CovarianceConstantsAutoTuner()            
    auto_tuner.main()