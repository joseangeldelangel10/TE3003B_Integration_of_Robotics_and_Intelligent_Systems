#!/usr/bin/env python
import rospy
import pandas as pd
import rospkg
import os
from point2PointController import Point2PointController
from geometry_msgs.msg import Twist, Pose2D, Pose
from nav_msgs.msg import Odometry
from mini_challenge_1.srv import *
import numpy as np

class PuzzlebotP2PExperiment():
    def __init__(self):        
        self.p2p_contr = Point2PointController()
        self.p2p_contr.goal = (0.0, 0.0, -np.pi/2)
        self.experiment_pose_sub = None
        self.csv_file_name = None
        self.data_saved = False
        self.nav_topic_d_type = None
        self.rate = rospy.Rate(20.0)                

    def reset_puzzlebot_sim(self):
        rospy.wait_for_service('reset_puzzlebot_sim')
        try:
            reset_sim = rospy.ServiceProxy('reset_puzzlebot_sim', ResetPuzzlebotSim)
            response = reset_sim()
            return response.success
        except rospy.ServiceException:
            print("reset sim service call failed")

    def reset_odometry(self):
        print("data type is {t}".format(t = self.nav_topic_d_type))
        if self.nav_topic_d_type == "<class 'nav_msgs.msg._Odometry.Odometry'>":
            rospy.wait_for_service('reset_odometry')
            try:
                reset_odom = rospy.ServiceProxy('reset_odometry', ResetOdometry)
                response = reset_odom()
                return response.success
            except rospy.ServiceException:
                print("reset odom service call failed")

    def define_nav_pose_topic(self, topic_name, topic_d_type):
        self.nav_topic_d_type = str(topic_d_type)
        self.p2p_contr.define_nav_pose_topic(topic_name, topic_d_type)

    def define_csv_file_to_save_experiment_data(self, csv_file_name):
        rp = rospkg.RosPack()
        package_path = rp.get_path('mini_challenge_1')
        self.csv_file_name = os.path.join(package_path, "experiments_data", csv_file_name)

    def define_experiment_pose_topic(self, pose_topic, topic_d_type):        
        self.experiment_pose_sub = rospy.Subscriber(pose_topic, topic_d_type, self.experiment_pose_callback)

    def experiment_pose_callback(self, msg):
        if self.p2p_contr.goal_reached and not self.data_saved:
            if self.csv_file_name is not None:
                if str(type(msg)) == "<class 'geometry_msgs.msg._Pose2D.Pose2D'>":            
                    self.save_resulting_goal(self.csv_file_name, (msg.x, msg.y, msg.theta))
                    self.data_saved = True                                    
                elif str(type(msg)) == "<class 'geometry_msgs.msg._Pose.Pose'>":
                    self.save_resulting_goal(self.csv_file_name, (msg.position.x, msg.position.y, msg.orientation.z))
                    self.data_saved = True
                else:
                    rospy.logerr("Error saving data!!!!!! -- invalid resulting pose info") 
            else:
                rospy.logerr("Error saving data!!!!!! -- please ensure csv file name is given")                        
        else:
            pass

    def save_resulting_goal(self, csv_file_name, resulting_pose_2d):
        df = pd.read_csv(csv_file_name)
        try:
            last_run_number = max(df["run_num"])
        except ValueError:
            last_run_number = 0
        new_data = {"run_num": [last_run_number + 1], 
                    "t_f":[None],
                    "x_goal":[self.p2p_contr.goal[0]], 
                    "y_goal":[self.p2p_contr.goal[1]], 
                    "th_goal":[self.p2p_contr.goal[2]], 
                    "x_f":[resulting_pose_2d[0]], 
                    "y_f":[resulting_pose_2d[1]], 
                    "th_f":[resulting_pose_2d[2]]}
        df2 = pd.DataFrame(new_data)
        df = df.append(df2, sort=True)
        df.to_csv(csv_file_name, index=False)        
        print("data saved to {d}".format(d = csv_file_name))

    def main(self):
        while not rospy.is_shutdown():
            self.p2p_contr.inherited_main()                
            self.rate.sleep()
            if self.data_saved:
                break        
        

if __name__ == '__main__':    
    total_experiments = 10
    completed_experiments = 0
    while (total_experiments - completed_experiments) > 0:     
        p2p_experiment = PuzzlebotP2PExperiment()
        p2p_experiment.define_nav_pose_topic("/odom", Odometry)
        p2p_experiment.define_experiment_pose_topic("/pose", Pose2D)
        p2p_experiment.define_csv_file_to_save_experiment_data("our_sim.csv")
        p2p_experiment.main()
        p2p_experiment.reset_puzzlebot_sim()
        p2p_experiment.reset_odometry()
        completed_experiments += 1