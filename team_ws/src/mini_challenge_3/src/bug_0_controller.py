#!/usr/bin/env python3

"""Made by:
    Leonardo Javier Nava
        navaleonardo40@gmail.com
    Jose Angel del Angel Dominguez
        joseangeldelangel10@gmail.com
    Raul López Musito
        raulmusito@gmail.com
Code description:
TO-DO - Implement Bug 0 algorithm
Notes:
"""

import math
import rospy
import nav_functions
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Bug0():
    def __init__(self, targetx, targety, wall_distance):
        
        rospy.init_node("bug_0_controller")
        rospy.Subscriber("/odom", Odometry, self.odom_callback)                
        
        self.scan_listener = rospy.Subscriber('/scan', LaserScan,self.scan_callback)
                
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)        
        self.vel_msg = Twist()

        self.target_postition_xy_2d = (targetx, targety)
        self.wall_distance = wall_distance
                
        self.current_position_xy_2d = (None, None)
        self.scan = None
        self.current_angle = None
        self.displaced_angle = 0.0
                     
        self.angular_error_treshold = 0.2
        self.distance_error_treshold = 0.08                    

        self.go2point_angular_kp = 0.2
        self.go2point_linear_kp = 0.2

        self.wall_kp_follow = 0.8
        self.wall_kp_avoid = 0.5

        self.v_max = 0.6
        self.w_max = 0.6


        self.state = "go_to_point"        

        self.rate = rospy.Rate(20)



    def scan_callback(self, msg):
        self.scan = msg    

    def reset_values(self):        
        self.vel_msg = Twist()        
        self.current_position_xy_2d = (None, None)
        self.target_postition_xy_2d = (None, None)
        self.current_angle = None        
        self.displaced_angle = 0.0

    def odom_callback(self, data):
        self.current_position_xy_2d = ( data.pose.pose.position.x , data.pose.pose.position.y )                             
        self.current_angle = nav_functions.calculate_yaw_angle_deg( data.pose.pose.orientation )

    def turn_left(self, p2p_target_angle, angle_to_rotate = 0.174533):
        
        if self.get_laser_value_at_angle(p2p_target_angle) > self.wall_distance:
                self.state = "go_to_point"
                self.displaced_angle = 0.0
        elif self.last_turn_time == None and self.state == "turn_left":
            self.last_turn_time = rospy.get_time()
        else:    
            if self.displaced_angle < angle_to_rotate:                
                linear_x = 0.0
                angular_z = self.w_max            
                self.vel_msg.linear.x = linear_x
                self.vel_msg.angular.z = angular_z                
                current_time = rospy.get_time()
                self.displaced_angle += abs(angular_z)*(current_time - self.last_turn_time)
                self.last_turn_time = current_time
            else:
                self.state = "follow_wall"
                self.displaced_angle = 0.0        
                self.last_turn_time = None            

    def go_to_point_controller(self, angle_error, distance_error):
        
        if distance_error <= self.distance_error_treshold:                                                            
            self.state = "arrived"
        elif abs(angle_error) > self.angular_error_treshold:                                    
            self.vel_msg.angular.z = nav_functions.saturate_signal(self.go2point_angular_kp*angle_error, self.w_max) 
        elif distance_error > self.distance_error_treshold:  
            if self.scan.ranges[0] <= self.wall_distance:
                self.state = "turn_left"
            else:
                self.vel_msg.linear.x = nav_functions.saturate_signal(self.go2point_linear_kp*distance_error, self.v_max)

    def get_laser_value_at_angle(self, angle_in_rads):
        angle_index = round( (angle_in_rads*2*math.pi)/len(self.scan.ranges) )
        return self.scan.ranges[int(angle_index)]

    def right_hand_rule_controller(self, p2p_target_angle):
        
        if self.scan != None:
            if self.get_laser_value_at_angle(p2p_target_angle) > self.wall_distance:
                self.state = "go_to_point"
            elif self.scan_ranges[0] <= self.wall_distance:
                self.state = "turn_left"                
            else:
                # TODO complete linear_x and angular_z vals
                dist_at_0 = self.scan.ranges[len(self.scan.ranges) // (3.0/4.0)]
                dist_at_45 = self.scan.ranges[len(self.scan.ranges) // (7.0/8.0)]
                l3 = np.sqrt( (dist_at_0**2.0) + (dist_at_45**2.0) - 2.0*dist_at_0*dist_at_45*(np.sqrt(2.0)/2.0) )
                y1 = np.arcsin((dist_at_45*(np.sqrt(2.0)/2.0))/l3)
                ang_err = y1 - np.pi/2.0            
                angular_z = self.wall_kp_follow*ang_err
                relative_euclidean_distance_to_wall = dist_at_0 - self.wall_dist

                if abs(angular_z) < 0.8: # TODO consider changing this condition or stating this tresh at init
                    if relative_euclidean_distance_to_wall <= (self.wall_dist/2):
                        angular_z += self.wall_kp_avoid*relative_euclidean_distance_to_wall
                    linear_x = self.v_max                
                else:                
                    linear_x = 0.0

                if dist_at_0 >= 1.2:
                    angular_z = -1.0 #turn_right

                self.vel_msg.angular.z = angular_z
                self.vel_msg.linear.x = linear_x
                
    def main(self):
        print("main inited node running")
        while not rospy.is_shutdown():            
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0
            if self.current_angle != None and self.scan != None:
                target_vector_minus_robot_vector = ( self.target_postition_xy_2d[0] - self.current_position_xy_2d[0],
                                                     self.target_postition_xy_2d[1] - self.current_position_xy_2d[1]  )
                target_angle = nav_functions.rad2deg(math.atan2( target_vector_minus_robot_vector[1],target_vector_minus_robot_vector[0]))                
                angle_error = nav_functions.calculate_angular_error_considering_upper_boundary_lag(target_angle, self.current_angle, "deg", (0.0,360.0))                
                distance_error = nav_functions.euclidean_distance_single_point_2d( target_vector_minus_robot_vector )                          
                if self.state == "go_to_point":
                    self.go_to_point_controller(angle_error, distance_error)
                elif self.state == "follow_wall":
                    self.right_hand_rule_controller(target_angle)  
                elif self.state == "turn_left":
                    self.turn_left(target_angle) 
                elif self.state == "arrived":
                    print("ia llegue")
                    exit             
                self.vel_pub.publish(self.vel_msg)
                
            self.rate.sleep()          

if __name__ == "__main__":
    bug_0 = Bug0(0.0,10.0,0.6)
    bug_0.main()
