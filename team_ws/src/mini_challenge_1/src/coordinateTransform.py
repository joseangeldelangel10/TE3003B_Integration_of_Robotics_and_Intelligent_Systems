#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Float32
import tf

class CoordinateTransform:
    
    def __init__(self):
        rospy.init_node('coordinate_transform')
        self.transform_broadcaster = tf.TransformBroadcaster()
        self.puzzlebot_pose_sub = rospy.Subscriber('/pose', Pose2D, self.pose_callback)
        self.puzzlebot_pose_x = None
        self.puzzlebot_pose_y = None
        self.puzzlebot_pose_theta = None   

        self.left_wheel_angle_sub = rospy.Subscriber('/left_wheel_angle', Float32, self.left_callback)
        self.right_wheel_angle_sub = rospy.Subscriber('/right_wheel_angle', Float32, self.right_callback)
        self.left_wheel_angle_ = None
        self.right_wheel_angle = None

        self.rate = rospy.Rate(10.0)            

    def pose_callback(self, msg):
        self.puzzlebot_pose_x = msg.x
        self.puzzlebot_pose_y = msg.y
        self.puzzlebot_pose_theta = msg.theta

    def left_callback(self, msg):
        self.left_wheel_angle = msg.data

    def right_callback(self, msg):
        self.right_wheel_angle = msg.data


    def main(self):
        while not rospy.is_shutdown():
            if self.puzzlebot_pose_x is not None and self.puzzlebot_pose_y is not None and self.puzzlebot_pose_theta is not None:
                
                self.transform_broadcaster.sendTransform((self.puzzlebot_pose_x, self.puzzlebot_pose_y, 0.0),
                                tf.transformations.quaternion_from_euler(0, 0, self.puzzlebot_pose_theta),
                                rospy.Time.now(),
                                "base_link",
                                "map")
                
                self.transform_broadcaster.sendTransform((0.05, 0.09, 0.0),
                                tf.transformations.quaternion_from_euler(0, self.left_wheel_angle, 0),
                                rospy.Time.now(),
                                "left_wheel",
                                "base_link")
                
                self.transform_broadcaster.sendTransform((0.05, -0.09, 0.0),
                                tf.transformations.quaternion_from_euler(0, self.right_wheel_angle,0 ),
                                rospy.Time.now(),
                                "right_wheel",
                                "base_link")
                
            self.rate.sleep()

if __name__ == '__main__':
    instance = CoordinateTransform()
    instance.main()