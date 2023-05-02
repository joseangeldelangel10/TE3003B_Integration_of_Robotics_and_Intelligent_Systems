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
        self.previous_err = None
        self.current_err = 0.0        
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
        cov_constants_file = open(cov_constants_file_name)
        cov_constants = json.load(cov_constants_file)        
        return (cov_constants["kr"], cov_constants["kl"])

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
            return response.success
        except rospy.ServiceException:
            print("Get puzzlebot covariance matix_on_simulation service call failed")  

    def get_puzzlebot_covariance_mat(self):
        rospy.wait_for_service('get_puzzlebot_covariance_mat')
        try:
            get_pbot_cov_mat = rospy.ServiceProxy('get_puzzlebot_covariance_mat', getPuzzlebotCovarianceMat)
            response = get_pbot_cov_mat(experiments_file,t,linear)
            return response.covariance_mat
        except rospy.ServiceException:
            print("Get puzzlebot covariance matrix service call failed")

    def main(self):
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

                        if self.previous_err is None:
                            self.previous_err = self.current_err
                            self.current_err = 0.0
                            #self.previous_kl =
                    else:
                        experiment_cov_matrix = None # TODO change this when service is done 
                        sim_cov_matrix = None # TODO change this when service is done
                        diff_mat = experiment_cov_matrix - sim_cov_matrix
                        self.current_err +=  (diff_mat**2).sum()
                    #if self.previous_kr is None and self.previous_kl is None:
                    # TODO implement autotuning logic
            self.rate.sleep()        
        

if __name__ == '__main__':    
    auto_tuner = CovarianceConstantsAutoTuner()            
    auto_tuner.main()