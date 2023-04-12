#!/usr/bin/env python
import rospy
import pandas as pd
import rospkg
import os
from point2PointController import Point2PointController

class PuzzlebotP2PExperiment(Point2PointController):
    def __init__(self):
        super().__init__()        
        self.experiment_pose_sub = None
        self.csv_file_name = None
        self.data_saved = False

    def define_csv_file_to_save_experiment_data(self, csv_file_name):
        rp = rospkg.RosPack()
        package_path = rp.get_path('mini_challenge_1')
        self.csv_file_name = os.path.join(package_path, "experiments_data", csv_file_name)

    def define_experiment_pose_topic(self, pose_topic, topic_d_type):        
        self.experiment_pose_sub = rospy.Subscriber(pose_topic, topic_d_type, self.experiment_pose_callback)

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

if __name__ == '__main__':
    pass