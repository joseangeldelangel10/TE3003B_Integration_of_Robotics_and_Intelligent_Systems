#!/usr/bin/env python
import rospy
import pandas as pd
import rospkg
import os
from point2PointController import Point2PointController
from geometry_msgs.msg import Twist, Pose2D, Pose
from nav_msgs.msg import Odometry

class PuzzlebotP2PExperiment():
    def __init__(self):        
        self.p2p_contr = Point2PointController()
        self.experiment_pose_sub = None
        self.csv_file_name = None
        self.data_saved = False
        self.rate = rospy.Rate(20.0)        

    def define_nav_pose_topic(self, topic_name, topic_d_type):
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
        last_run_number = max(df["run_num"])
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
        

if __name__ == '__main__':
    p2p_experiment = PuzzlebotP2PExperiment()
    p2p_experiment.define_nav_pose_topic("/pose", Pose2D)
    p2p_experiment.define_experiment_pose_topic("/pose", Pose2D)
    p2p_experiment.define_csv_file_to_save_experiment_data("our_sim.csv")
    p2p_experiment.main()