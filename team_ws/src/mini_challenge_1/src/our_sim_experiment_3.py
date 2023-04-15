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

class PuzzlebotOpenLoopExperiment():
    def __init__(self):        
        rospy.init_node('puzzlebot_experiment')
        #self.p2p_contr = Point2PointController()
        self.linear_vel, self.angular_vel, self.exec_time = (0.0, np.pi/5, 3.0)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.cmd_vel_msg = Twist()
        self.experiment_pose_sub = None
        self.csv_file_name = None
        self.data_saved = False
        self.first_time = True
        self.inital_time = None                
        #self.nav_topic_d_type = None
        self.rate = rospy.Rate(20.0)

    def publish_vel(self, linear_vel, angular_vel):
        self.cmd_vel_msg.linear.x = linear_vel
        self.cmd_vel_msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(self.cmd_vel_msg)                

    def reset_puzzlebot_sim(self):
        rospy.wait_for_service('reset_puzzlebot_sim')
        try:
            reset_sim = rospy.ServiceProxy('reset_puzzlebot_sim', ResetPuzzlebotSim)
            response = reset_sim()
            return response.success
        except rospy.ServiceException:
            print("reset sim service call failed")

    def define_csv_file_to_save_experiment_data(self, csv_file_name):
        rp = rospkg.RosPack()
        package_path = rp.get_path('mini_challenge_1')
        self.csv_file_name = os.path.join(package_path, "experiments_data", csv_file_name)

    def define_experiment_pose_topic(self, pose_topic, topic_d_type):        
        self.experiment_pose_sub = rospy.Subscriber(pose_topic, topic_d_type, self.experiment_pose_callback)

    def experiment_pose_callback(self, msg):
        if self.inital_time is not None and (rospy.get_time() - self.inital_time) >= self.exec_time and not self.data_saved:
            if self.csv_file_name is not None:
                if str(type(msg)) == "<class 'geometry_msgs.msg._Pose2D.Pose2D'>":            
                    self.save_resulting_goal(self.csv_file_name, (msg.x, msg.y, msg.theta))
                    self.data_saved = True
                    self.publish_vel(0.0, 0.0)                                                            
                elif str(type(msg)) == "<class 'geometry_msgs.msg._Pose.Pose'>":
                    self.save_resulting_goal(self.csv_file_name, (msg.position.x, msg.position.y, msg.orientation.z))
                    self.data_saved = True
                    self.publish_vel(0.0, 0.0)                        
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
                    "t_f":[self.exec_time],                    
                    "x_f":[resulting_pose_2d[0]], 
                    "y_f":[resulting_pose_2d[1]], 
                    "th_f":[resulting_pose_2d[2]],
                    "linear_vel": [self.linear_vel], 
                    "ang_vel":[self.angular_vel]}        
        df2 = pd.DataFrame(new_data)
        df = df.append(df2, sort=True)
        df.to_csv(csv_file_name, index=False)        
        print("data saved to {d}".format(d = csv_file_name))

    def main(self):
        while not rospy.is_shutdown():                            
            # TODO: implement this            
            if self.data_saved:
                break
            else:
                if self.first_time:
                    self.inital_time = rospy.get_time()
                    self.first_time = False
                else:
                    if (rospy.get_time() - self.inital_time) < self.exec_time:
                        self.publish_vel(self.linear_vel, self.angular_vel)
                    else:
                        self.publish_vel(0.0, 0.0)                        
            self.rate.sleep()        
        

if __name__ == '__main__':    
    total_experiments = 10
    completed_experiments = 0
    while (total_experiments - completed_experiments) > 0:     
        puzz_ol_experiment = PuzzlebotOpenLoopExperiment()        
        puzz_ol_experiment.define_experiment_pose_topic("/pose", Pose2D)
        puzz_ol_experiment.define_csv_file_to_save_experiment_data("open_loop_experiments/our_sim.csv")
        puzz_ol_experiment.main()
        puzz_ol_experiment.reset_puzzlebot_sim()        
        completed_experiments += 1