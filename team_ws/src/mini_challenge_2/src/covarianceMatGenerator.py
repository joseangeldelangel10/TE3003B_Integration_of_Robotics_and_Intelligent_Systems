#!/usr/bin/env python
import rospy
import pandas as pd
import rospkg
import os
import numpy as np

class CovarianceMatrixGenerator():
    def __init__(self):      
        rospy.init_node('covariance_mat_generator')  
        self.csv_file_name = self.define_csv_file_with_experiment_data("dummy_experiments_for_testing/physical.csv")    
        self.matrix_generated = False
        self.rate = rospy.Rate(20.0)                

    def define_csv_file_with_experiment_data(self, csv_file_name):
        rp = rospkg.RosPack()
        package_path = rp.get_path('mini_challenge_1')
        final_csv_file_name = os.path.join(package_path, "experiments_data", csv_file_name)
        return final_csv_file_name

    def get_covariance_mat(self):
        df = pd.read_csv(self.csv_file_name)
        df = df[df["x_f"] > 0.5]
        df = df[df["x_f"] < 1.5]
        df = df[["x_f", "y_f", "th_f"]]
        covariance_mat = np.cov(df.to_numpy() , rowvar=False)
        
        df_only_x = df["x_f"].to_numpy()
        n_samples = len(df_only_x)
        x_mean = df_only_x.mean()
        df_only_x = df_only_x.reshape((n_samples, 1)) - x_mean
        x_variance = np.dot(df_only_x.T, df_only_x)/(n_samples-1)
        #df2 = pd.DataFrame(new_data)
        #df = df.append(df2, sort=True)        
        print("data frame \n {d}".format(d = df))
        print("covariance mat \n {m}".format(m = covariance_mat))
        print("x var \n {x}".format(x = x_variance))
        self.matrix_generated = True

    def main(self):
        while not rospy.is_shutdown():
            self.get_covariance_mat()
            self.rate.sleep()
            if self.matrix_generated:
                break                    
        

if __name__ == '__main__':    
    cov_mat_generator = CovarianceMatrixGenerator()
    cov_mat_generator.main()