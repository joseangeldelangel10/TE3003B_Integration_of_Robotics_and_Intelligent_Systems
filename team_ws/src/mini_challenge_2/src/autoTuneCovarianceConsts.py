#!/usr/bin/env python
import rospy
import pandas as pd
import rospkg
import os
from geometry_msgs.msg import Twist, Pose2D, Pose
from nav_msgs.msg import Odometry
from mini_challenge_1.srv import *
from mini_challenge_2.srv import *
import numpy as np
import json
import random

class CovarianceConstantsAutoTuner():
    def __init__(self):        
        rospy.init_node('auto_tune_covariance_constants')
        self.epochs = 10
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
        cov_constants_file.close()        
        return (cov_constants["kr"], cov_constants["kl"])
    
    def rewrite_covariance_constants(self, new_kr, new_kl, error):        
        if new_kr is np.nan:
            raise Exception("REACHED A NAN VALUE")
        rp = rospkg.RosPack()
        this_pkg_path = rp.get_path('mini_challenge_2')
        cov_constants_file_name = os.path.join(this_pkg_path, "src", "constants", "covariance_prop_constants.json")
        cov_constants_file = open(cov_constants_file_name, "r")        
        cov_constants = json.load(cov_constants_file)    
        least_err = cov_constants["least_error"]
        cov_constants_file.close()
        cov_constants["kr"] = new_kr
        cov_constants["kl"] = new_kl
        if error is not None and least_err > np.log(error):
            cov_constants["least_error"] = np.log(error)
            cov_constants["kr_le"] = new_kr
            cov_constants["kl_le"] = new_kl
        cov_constants_file_w = open(cov_constants_file_name, "w")
        json.dump(cov_constants, cov_constants_file_w)
        cov_constants_file.close()                        

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

    def get_puzzlebot_covariance_mat_on_sim(self):
        rospy.wait_for_service('get_puzzlebot_covariance_mat_on_sim')
        try:
            get_pbot_cov_mat_on_sim = rospy.ServiceProxy('get_puzzlebot_covariance_mat_on_sim', GetPuzzlebotCovarianceMatOnSim)
            response = get_pbot_cov_mat_on_sim()
            return response.covariance_mat.data
        except rospy.ServiceException:
            print("Get puzzlebot covariance matix_on_simulation service call failed")  

    def get_puzzlebot_covariance_mat(self, experiments_file, t, linear):
        rospy.wait_for_service('get_puzzlebot_covariance_mat')
        try:
            get_pbot_cov_mat = rospy.ServiceProxy('get_puzzlebot_covariance_mat', getPuzzlebotCovarianceMat)
            response = get_pbot_cov_mat(experiments_file,t,linear)
            return response.covariance_mat.data
        except rospy.ServiceException:
            print("Get puzzlebot covariance matrix service call failed")

    def main(self):
        for i in range(self.epochs):
            while not rospy.is_shutdown():
                linear_vel, ang_vel, exec_time = self.experiments_linear_vel_ang_vel_and_exec_time[self.experiment_index]                                        
                if self.first_time:
                    self.inital_time = rospy.get_time()
                    self.first_time = False
                else:
                    if (rospy.get_time() - self.inital_time) < exec_time:                        
                        self.publish_vel(linear_vel, ang_vel)
                    else:
                        self.publish_vel(0.0, 0.0)
                        print("experiment_index is {i}".format(i = self.experiment_index))
                        if self.experiment_index >= (len(self.experiments_linear_vel_ang_vel_and_exec_time) -1):
                            if linear_vel != 0.0:
                                experiment_cov_matrix = np.array( self.get_puzzlebot_covariance_mat("open_loop_experiments/physical.csv",exec_time, linear=True) ) 
                            else:
                                experiment_cov_matrix = np.array( self.get_puzzlebot_covariance_mat("open_loop_experiments/physical.csv",exec_time, linear=False) ) 
                            sim_cov_matrix = np.array( self.get_puzzlebot_covariance_mat_on_sim() )
                            diff_mat = experiment_cov_matrix - sim_cov_matrix
                            print("diff mat is {d}".format(d=diff_mat))
                            self.current_err +=  (diff_mat**2).sum()

                            self.current_kr, self.current_kl = self.get_covariance_constants()
                            if self.previous_err is None:                                                        
                                new_kr = self.current_kr + random.uniform(-0.2, 0.2)
                                new_kl = self.current_kl + random.uniform(-0.2, 0.2)
                                self.rewrite_covariance_constants(new_kr, new_kl, self.current_err)
                            else:
                                kr_error_derivative = (self.current_err - self.previous_err)/(self.current_kr - self.previous_kr)
                                kl_error_derivative = (self.current_err - self.previous_err)/(self.current_kl - self.previous_kl)
                                new_kr = self.current_kr - self.learning_rate*kr_error_derivative
                                new_kl = self.current_kl - self.learning_rate*kl_error_derivative                                
                                self.rewrite_covariance_constants(new_kr, new_kl, self.current_err)
                            self.previous_err = self.current_err
                            self.previous_kr, self.previous_kl = (self.current_kr, self.current_kl)
                            self.current_err = 0.0
                            self.first_time = True
                            self.inital_time = None
                            self.experiment_index = 0
                            rospy.sleep(1.5)
                            self.reset_puzzlebot_sim()
                            self.reset_odometry()
                            break
                        else:
                            if linear_vel != 0.0:
                                experiment_cov_matrix = np.array( self.get_puzzlebot_covariance_mat("open_loop_experiments/physical.csv",exec_time, linear=True) ) 
                            else:
                                experiment_cov_matrix = np.array( self.get_puzzlebot_covariance_mat("open_loop_experiments/physical.csv",exec_time, linear=False) ) 
                            sim_cov_matrix = np.array( self.get_puzzlebot_covariance_mat_on_sim() )                            
                            diff_mat = experiment_cov_matrix - sim_cov_matrix
                            self.current_err +=  (diff_mat**2).sum()
                            self.first_time = True
                            self.inital_time = None
                            self.experiment_index += 1
                            self.reset_puzzlebot_sim()
                            self.reset_odometry()                                                
                self.rate.sleep()        
        

if __name__ == '__main__':    
    auto_tuner = CovarianceConstantsAutoTuner()            
    auto_tuner.main()