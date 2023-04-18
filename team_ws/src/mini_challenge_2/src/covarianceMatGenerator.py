#!/usr/bin/env python
import rospy
import pandas as pd
import rospkg
import os
import numpy as np
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from mini_challenge_2.srv import *

class CovarianceMatrixGenerator():
    def __init__(self):      
        rospy.init_node('covariance_mat_generator')  
        self.csv_file_name = self.define_csv_file_with_experiment_data("dummy_experiments_for_testing/physical.csv")    
        self.get_covariance_mat_server = rospy.Service("get_puzzlebot_covariance_mat", getPuzzlebotCovarianceMat, self.get_covariance_mat)
        self.get_covariance_mat_result = Float64MultiArray()
        self.covariance_mat_height_layout = MultiArrayDimension()
        self.covariance_mat_width_layout = MultiArrayDimension()
        
        self.covariance_mat_height_layout.label = "height"
        self.covariance_mat_height_layout.size = 3
        self.covariance_mat_height_layout.stride = 3*3
        self.covariance_mat_width_layout.label = "width"
        self.covariance_mat_width_layout.size = 3
        self.covariance_mat_width_layout.stride = 3
        
        self.rate = rospy.Rate(20.0)                

    def define_csv_file_with_experiment_data(self, csv_file_name):
        rp = rospkg.RosPack()
        package_path = rp.get_path('mini_challenge_1')
        final_csv_file_name = os.path.join(package_path, "experiments_data", csv_file_name)
        return final_csv_file_name
    
    def get_single_variable_variance(self, df, variable):
        df_only_x = df[variable].to_numpy()
        n_samples = len(df_only_x)
        x_mean = df_only_x.mean()
        df_only_x = df_only_x.reshape((n_samples, 1)) - x_mean
        x_variance = np.dot(df_only_x.T, df_only_x)/(n_samples-1)        
        return x_variance

    def get_covariance_mat(self, req):
        if req.experiments_file == "":
            df = pd.read_csv(self.csv_file_name)
        else:
            df = pd.read_csv( self.define_csv_file_with_experiment_data(req.experiments_file) )

        if req.linear:
            df = df[df["ang_vel"] == 0.0]
        else:
            df = df[df["linear_vel"] == 0.0]

        if req.t == 0.0:
            if req.linear:
                df = df[df["t_f"] == 6.0]
            else:
                df = df[df["t_f"] == 3.0]
        else:
            df = df[df["t_f"] == req.t]                                

        df = df[["x_f", "y_f", "th_f"]]
        covariance_mat = np.cov(df.to_numpy() , rowvar=False)
                
        rospy.loginfo("COVARIANCE MATRIX REQUESTED")
        rospy.loginfo("respose prove: \t x_variance_in_matrix={v1} \t x_variance_manual={v2}".format(
            v1 = covariance_mat[0][0],
            v2 = self.get_single_variable_variance(df, "x_f")
        ))        

        cov_mat_n, cov_mat_m = covariance_mat.shape
        self.get_covariance_mat_result.layout.dim = [self.covariance_mat_height_layout, self.covariance_mat_width_layout]        
        self.get_covariance_mat_result.layout.data_offset = 0
        self.get_covariance_mat_result.data = covariance_mat.reshape((cov_mat_m*cov_mat_n,)) 
        return getPuzzlebotCovarianceMatResponse(self.get_covariance_mat_result)

    def main(self):
        while not rospy.is_shutdown():
            rospy.spin()                            

if __name__ == '__main__':    
    cov_mat_generator = CovarianceMatrixGenerator()
    cov_mat_generator.main()