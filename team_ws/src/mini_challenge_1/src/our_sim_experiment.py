#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose2D, Pose
import pandas as pd
import rospkg
import os

class Point2PointController:
    
    def __init__(self):
        rospy.init_node('point_2_point_controller')
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)        
        self.nav_pose_sub = None
        self.linear_err = None
        self.p2p_angular_err = None
        self.angular_err = None
        self.goal = (1.0, 0.0, 0.0)
        self.linear_tresh = 0.05
        self.p2p_angular_tresh = 0.05
        self.angular_tresh = 0.05
        self.kpv = 1
        self.kpw = 1
        self.kpw2 = 1
        self.vel_msg = Twist()
        self.rate = rospy.Rate(10.0)
        self.goal_reached = False
        
        self.data_saved = False
        self.experiment_pose_sub = None
        self.csv_file_name = None

    def define_csv_file_to_save_experiment_data(self, csv_file_name):
        rp = rospkg.RosPack()
        package_path = rp.get_path('mini_challenge_1')
        self.csv_file_name = os.path.join(package_path, "experiments_data", csv_file_name)

    def define_experiment_pose_topic(self, pose_topic, topic_d_type):        
        self.experiment_pose_sub = rospy.Subscriber(pose_topic, topic_d_type, self.experiment_pose_callback)
        
    def define_nav_pose_topic(self, pose_topic, topic_d_type):        
        self.nav_pose_sub = rospy.Subscriber(pose_topic, topic_d_type, self.nav_pose_callback)

    def experiment_pose_callback(self, msg):
        if self.goal_reached and not self.data_saved:
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

    def nav_pose_callback(self, msg):         
        if str(type(msg)) == "<class 'geometry_msgs.msg._Pose2D.Pose2D'>":            
            self.linear_err = np.sqrt( (self.goal[0] - msg.x)**2 + (self.goal[1] - msg.y)**2 )
            self.p2p_angular_err = (np.arctan2(self.goal[1], self.goal[0])) - msg.theta
            self.angular_err = self.goal[2] - msg.theta                        
        elif str(type(msg)) == "<class 'geometry_msgs.msg._Pose.Pose'>":
            self.linear_err = np.sqrt( (self.goal[0] - msg.position.x)**2 + (self.goal[1] - msg.position.y)**2 )
            self.p2p_angular_err = (np.arctan2(self.goal[1], self.goal[0])) - msg.orientation.z
            self.angular_err = self.goal[2] - msg.theta

    def save_resulting_goal(self, csv_file_name, resulting_pose_2d):
        df = pd.read_csv(csv_file_name)
        last_run_number = max(df["run_num"])
        new_data = {"run_num": [last_run_number + 1], 
                    "t_f":[None],
                    "x_goal":[self.goal[0]], 
                    "y_goal":[self.goal[1]], 
                    "th_goal":[self.goal[2]], 
                    "x_f":[resulting_pose_2d[0]], 
                    "y_f":[resulting_pose_2d[1]], 
                    "th_f":[resulting_pose_2d[2]]}
        df2 = pd.DataFrame(new_data)
        df = df.append(df2, sort=True)
        df.to_csv(csv_file_name, index=False)        
        rospy.loginfo("data saved to {d}".format(d = csv_file_name))

    def pid2D(self):
                      
        if abs(self.linear_err) >= self.linear_tresh:
            v = self.kpv*self.linear_err
            if abs(self.p2p_angular_err) >= self.p2p_angular_tresh:
                w = self.kpw*self.p2p_angular_err
            else:
                w = 0.0
        else:
            v = 0.0
            if abs(self.angular_err) >= self.angular_tresh:
                w = self.kpw2*self.angular_err
            else:
                w = 0.0
                 
        if abs(v) >= 0.2:
            v = np.sign(v)*0.19
        if abs(w) >= np.pi/5:
            w = np.sign(w)*np.pi/5 - (np.sign(w)*0.05)

  
        if abs(self.angular_err) < self.angular_tresh and abs(self.linear_err) < self.linear_tresh:
            print("goal reached")
            self.goal_reached = True

        return v,w


    def main(self):
        while not rospy.is_shutdown():
            if self.linear_err is not None and self.angular_err is not None:            
                v_pid,w_pid = self.pid2D()                
                self.vel_msg.linear.x = v_pid
                self.vel_msg.angular.z = w_pid
                self.vel_pub.publish(self.vel_msg)                
            self.rate.sleep()

if __name__ == '__main__':
    ddm = Point2PointController()
    ddm.define_nav_pose_topic("/pose", Pose2D)
    ddm.define_experiment_pose_topic("/pose", Pose2D)
    ddm.define_csv_file_to_save_experiment_data("our_sim.csv")
    ddm.main()