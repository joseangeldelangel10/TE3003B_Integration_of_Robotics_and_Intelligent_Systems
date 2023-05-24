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
    def __init__(self, targets,  wall_distance):
        
        rospy.init_node("bug_0_controller")
        rospy.Subscriber("/kalman_corrected_odom", Odometry, self.odom_callback)                
        
        self.scan_listener = rospy.Subscriber('/scan', LaserScan,self.scan_callback)
                
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)        
        self.vel_msg = Twist()

        self.targets = targets
        self.index = 0
        self.target_postition_xy_2d = self.targets[self.index]
        self.wall_distance = wall_distance

        print("Target is :", self.target_postition_xy_2d)
                
        self.current_position_xy_2d = (None, None)
        self.scan = None
        self.current_angle = None
        self.displaced_angle = 0.0
                     
        self.angular_error_treshold = 0.3    
        self.distance_error_treshold = 0.08                    

        self.go2point_angular_kp = 0.08
        self.go2point_linear_kp = 0.3

        self.v_max = 0.4        
        self.w_max = 0.6

        self.puzzlebot_passing_diameter = 0.60 # meters

        self.state = "go_to_point"                

        self.rate_val = 20.0
        self.rate = rospy.Rate(self.rate_val)        

        self.right_scan_angle = None
        self.front_right_scan_angle = None
        self.front_scan_angle = None
        self.front_left_scan_angle = None
        self.left_scan_angle = None

        self.right_scan_value = None
        self.front_right_scan_value = None
        self.front_scan_value = None
        self.front_left_scan_value = None
        self.left_scan_value = None

        self.angle_to_turn = None    
        self.angle_to_turn_is_computed = False
        self.last_turn_time = None

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

    def get_angle_to_turn(self):
        """
        self.get_ray_sectors_info()
        p2_x = self.front_left_scan_value*np.cos( np.deg2rad(self.front_left_scan_angle) )
        p2_y = self.front_left_scan_value*np.sin( np.deg2rad(self.front_left_scan_angle) )
        p1_x = self.front_right_scan_value*np.cos( np.deg2rad(self.front_right_scan_angle) )
        p1_y = self.front_right_scan_value*np.sin( np.deg2rad(self.front_right_scan_angle) )
        if p2_x - p1_x != 0.0:
            angle_to_turn = np.arctan2( (p2_y-p1_y), (p2_x - p1_x))
        else:
            angle_to_turn = np.pi/2.0

        #self.angle_to_turn = abs(angle_to_turn)
        """
        self.angle_to_turn = np.pi/2.0
        self.angle_to_turn_is_computed = True
        self.displaced_angle = 0.0
        self.last_turn_time = rospy.get_time()

    def turn_left(self):
        # TODO when we pass this to rover change logic so that the loop is closed using imu data
        if self.scan != None:            
            if not self.angle_to_turn_is_computed:
                self.get_angle_to_turn()
                print("angle to turn is: {a}".format(a = nav_functions.rad2deg(self.angle_to_turn)))
            else:                                    
                if self.displaced_angle < self.angle_to_turn:                
                    self.vel_msg.linear.x = 0.0
                    self.vel_msg.angular.z = self.w_max                                                                    
                    current_time = rospy.get_time()
                    self.displaced_angle += abs(self.vel_msg.angular.z)*(current_time - self.last_turn_time)
                    self.last_turn_time = current_time
                else:                
                    self.vel_msg.linear.x = 0.0
                    self.vel_msg.angular.z = 0.0
                    self.state = "follow_wall"

                    self.angle_to_turn = None    
                    self.angle_to_turn_is_computed = False
                    self.displaced_angle = 0.0
                    self.last_turn_time = None

    def go_to_point_controller(self, angle_error, distance_error):
        if distance_error <= self.distance_error_treshold:                                                            
            self.state = "arrived"
        elif self.obstacle_in_front():
                self.state = "turn_left"
        elif abs(angle_error) > self.angular_error_treshold:                                    
            self.vel_msg.angular.z = nav_functions.saturate_signal(self.go2point_angular_kp*angle_error, self.w_max) 
        elif distance_error > self.distance_error_treshold:              
            self.vel_msg.linear.x = nav_functions.saturate_signal(self.go2point_linear_kp*distance_error, self.v_max)    
    
    def get_laser_index_from_angle(self, angle_in_deg):        
        angle_in_deg = nav_functions.angle_to_only_possitive_deg(angle_in_deg)        
        angle_index = round( (angle_in_deg*len(self.scan.ranges))/360.0 )
        return int(angle_index)
    
    def get_values_at_target(self, fov_center, fov_range):        
        if fov_range > 180.0:
            Exception("fov is too large")
        
        if fov_center - (fov_range/2.0) < 0.0:
            print("entered into rad limit lower DEG, fov center is: {c}, fov is: {f}".format(c = fov_center, f = fov_range))
            values_at_fov_1 = self.scan.ranges[self.get_laser_index_from_angle(nav_functions.angle_to_only_possitive_deg(fov_center - (fov_range/2.0))) : ]
            values_at_fov_2 = self.scan.ranges[: self.get_laser_index_from_angle( fov_center + (fov_range/2.0) ) + 1]
            values_at_fov = values_at_fov_1 + values_at_fov_2
        elif fov_center + (fov_range/2.0) > 360.0:
            print("entered into rad limit upper DEG, fov center is: {c}, fov is: {f}".format(c = fov_center, f = fov_range))
            values_at_fov_1 = self.scan.ranges[self.get_laser_index_from_angle(fov_center - (fov_range/2.0)) : ]
            values_at_fov_2 = self.scan.ranges[: self.get_laser_index_from_angle( (fov_center + (fov_range/2.0))%360.0 ) + 1]
            values_at_fov = values_at_fov_1 + values_at_fov_2
        else:
            values_at_fov = self.scan.ranges[ self.get_laser_index_from_angle(fov_center-(fov_range/2.0)) : self.get_laser_index_from_angle(fov_center+(fov_range/2.0)) + 1 ]
        
        return values_at_fov
    
    def target_path_is_clear(self, p2p_target_angle):
        wall_dist_fov = 2.0*( 90.0 - nav_functions.rad2deg(np.arctan(self.wall_distance/(self.puzzlebot_passing_diameter/2.0))) )        
        target_direction = nav_functions.angle_to_only_possitive_deg(p2p_target_angle - self.current_angle)        
        values_at_target = self.get_values_at_target(target_direction, wall_dist_fov)        
        
        target_is_clear = ( (min(values_at_target) >= 2.25*self.wall_distance) and 
        ((target_direction > 0.0 and target_direction < 90.0) or 
        ((target_direction > 270.0) and target_direction < 360.0)) )

        return target_is_clear        
    
    def obstacle_in_front(self):
        wall_dist_fov = 2.0*( 90.0 - nav_functions.rad2deg(np.arctan(self.wall_distance/(self.puzzlebot_passing_diameter/2.0))) )                
        values_in_front = self.get_values_at_target(0.0, wall_dist_fov)
        #side_dists_to_obstacle = self.get_values_at_target(315.0, 30.0) # TODO check this, this should probably be deleted 
        return min(values_in_front) <= self.wall_distance
    
    """
    def right_hand_rule_controller(self, p2p_target_angle):
        
        if self.scan != None:
            self.get_ray_sectors_info()
            if self.front_scan_value <= 1.5*self.wall_distance:
                self.state = "turn_left"
            elif self.target_path_is_clear(p2p_target_angle):
                self.state = "go_to_point"
            elif self.right_scan_value > 2.5*self.wall_distance:
                self.vel_msg.angular.z = -self.v_max/self.wall_distance
                self.vel_msg.linear.x = self.v_max
            else:
                tangent_vect_p1 = np.array((
                    [[self.right_scan_value*np.cos( np.deg2rad(self.right_scan_angle) )],
                    [self.right_scan_value*np.sin( np.deg2rad(self.right_scan_angle) )]]
                ))
                tangent_vect_p2 = np.array((
                    [[self.front_right_scan_value*np.cos(np.deg2rad(self.front_right_scan_angle))],
                    [self.front_right_scan_value*np.sin(np.deg2rad(self.front_right_scan_angle))]]
                ))
                tangent_vect = tangent_vect_p2 - tangent_vect_p1
                print("tangent_vect val is \n {v}".format(v = tangent_vect))
                tangent_vect_unitary = tangent_vect/self.get_vector_mag(tangent_vect)            
                perpendicular_vect_to_robot = tangent_vect_p1 - (np.dot( tangent_vect_p1.reshape(2,), tangent_vect_unitary.reshape((2,)) ) * tangent_vect_unitary )
                perpendicular_vect_to_robot_unitary = perpendicular_vect_to_robot/self.get_vector_mag(perpendicular_vect_to_robot)
                wall_dist_error_vect = perpendicular_vect_to_robot - self.wall_distance*perpendicular_vect_to_robot_unitary

                parallel_kp = 0.8
                wall_dist_kp = 0.2

                follow_wall_vect = wall_dist_kp*wall_dist_error_vect + parallel_kp*tangent_vect 

                self.vel_msg.angular.z = np.arctan2( follow_wall_vect[1], follow_wall_vect[0] )            
                self.vel_msg.linear.x = self.v_max

    """

    def get_values_at_target_in_rad(self, fov_center, fov_range):        
        if fov_range > np.pi:
            Exception("fov is too large")        
        fov_center = nav_functions.angle_to_only_possitive(fov_center)
        if fov_center - (fov_range/2.0) < 0.0:
            print("entered into rad limit lower RAD, fov center is: {c}, fov is: {f}".format(c = fov_center, f = fov_range))
            values_at_fov_1 = self.scan.ranges[self.get_laser_index_from_angle_in_rad(nav_functions.angle_to_only_possitive(fov_center - (fov_range/2.0))) : ]
            values_at_fov_2 = self.scan.ranges[: self.get_laser_index_from_angle_in_rad( fov_center + (fov_range/2.0) )]
            values_at_fov = values_at_fov_1 + values_at_fov_2
        elif fov_center + (fov_range/2.0) > (2.0*np.pi):
            print("entered into rad limit upper RAD, fov center is: {c}, fov is: {f}".format(c = fov_center, f = fov_range))
            values_at_fov_1 = self.scan.ranges[self.get_laser_index_from_angle_in_rad(fov_center - (fov_range/2.0)) : ]
            values_at_fov_2 = self.scan.ranges[: self.get_laser_index_from_angle_in_rad( (fov_center + (fov_range/2.0))%(2.0*np.pi) )]
            values_at_fov = values_at_fov_1 + values_at_fov_2
        else:
            values_at_fov = self.scan.ranges[ self.get_laser_index_from_angle_in_rad(fov_center-(fov_range/2.0)) : self.get_laser_index_from_angle_in_rad(fov_center+(fov_range/2.0)) ]
        
        return values_at_fov

    def right_hand_rule_controller(self, p2p_target_angle):
        if self.scan != None:
            self.get_ray_sectors_info()
            if self.front_scan_value <= 1.5*self.wall_distance:
                self.state = "turn_left"
            elif self.target_path_is_clear(p2p_target_angle):
                self.state = "go_to_point"
            elif self.right_scan_value > 2.5*self.wall_distance:
                self.vel_msg.angular.z = -self.v_max/self.wall_distance
                self.vel_msg.linear.x = self.v_max
            else:
                tangent_vect_p1 = np.array((
                    [[self.right_scan_value*np.cos(self.right_scan_angle)],
                    [self.right_scan_value*np.sin(self.right_scan_angle)]]
                ))
                tangent_vect_p2 = np.array((
                    [[self.front_right_scan_value*np.cos(self.front_right_scan_angle)],
                    [self.front_right_scan_value*np.sin(self.front_right_scan_angle)]]
                ))
                tangent_vect = tangent_vect_p2 - tangent_vect_p1
                print("tangent_vect val is \n {v}".format(v = tangent_vect))
                tangent_vect_unitary = tangent_vect/self.get_vector_mag(tangent_vect)            
                perpendicular_vect_to_robot = tangent_vect_p1 - ( np.dot( tangent_vect_p1.reshape(2,), tangent_vect_unitary.reshape((2,)) ) * tangent_vect_unitary )
                perpendicular_vect_to_robot_unitary = perpendicular_vect_to_robot/self.get_vector_mag(perpendicular_vect_to_robot)
                wall_dist_error_vect = perpendicular_vect_to_robot - self.wall_distance*perpendicular_vect_to_robot_unitary

                parallel_kp = 0.5
                wall_dist_kp = 0.5

                follow_wall_vect = wall_dist_kp*wall_dist_error_vect + parallel_kp*tangent_vect 

                self.vel_msg.angular.z = np.arctan2( follow_wall_vect[1], follow_wall_vect[0] )            
                self.vel_msg.linear.x = self.v_max                

    def get_laser_index_from_angle_in_rad(self, angle_in_rad):
        angle_in_rad = nav_functions.angle_to_only_possitive(angle_in_rad)        
        angle_index = round( (angle_in_rad*len(self.scan.ranges))/(2.0*np.pi) )
        return int(angle_index)  
                
    def get_vector_mag(self, vect):
        return np.sqrt(vect[0]**2 + vect[1]**2)                

    def get_ray_sectors_info(self):

        if self.scan is not None:
            scan_angle_min = (-np.pi/2.0)
            scan_angle_max = np.pi/2.0                           
            sector_size = ( scan_angle_max - scan_angle_min)/5.0
            
            self.right_scan_angle = scan_angle_min + (1.0/2.0)*sector_size
            self.front_right_scan_angle = scan_angle_min + (3.0/2.0)*sector_size
            self.front_scan_angle = scan_angle_min + (5.0/2.0)*sector_size
            self.front_left_scan_angle = scan_angle_min + (7.0/2.0)*sector_size
            self.left_scan_angle = scan_angle_min + (9.0/2.0)*sector_size

            right_scan_values = self.get_values_at_target_in_rad(self.right_scan_angle, sector_size)            
            front_right_scan_values = self.get_values_at_target_in_rad(self.front_right_scan_angle, sector_size)
            front_scan_values = self.get_values_at_target_in_rad(self.front_scan_angle, sector_size)
            front_left_scan_values = self.get_values_at_target_in_rad(self.front_left_scan_angle, sector_size)
            left_scan_values = self.get_values_at_target_in_rad(self.left_scan_angle, sector_size)
            
            #print("RIGHT SCAN VALUES ARE: {v} ".format(v = right_scan_values))
            print("RIGHT SCAN VALUES len is: {}".format(len(right_scan_values)))
            #print("FRON RIGHT SCAN VALUES ARE: {v} ".format(v = front_right_scan_values))
            print("FRON RIGHT SCAN VALUES len is: {}".format(len(front_right_scan_values)))
            #print("FRONT SCAN VALUES ARE: {v} ".format(v = front_scan_values))
            print("FRONT SCAN len is: {}".format(len(front_scan_values)))
            #print("FRONT LEFT SCAN VALUES ARE: {v} ".format(v = front_left_scan_values))
            print("FRONT LEFT SCAN VALUES len is: {}".format(len(front_left_scan_values)))
            #print("LEFT SCAN VALUES ARE: {v} ".format(v = left_scan_values))
            print("LEFT SCAN len is: {}".format(len(left_scan_values)))

            if min(right_scan_values) == np.inf:
                self.right_scan_value = 12.0
            else:
                self.right_scan_value = min(right_scan_values)

            if min(front_right_scan_values) == np.inf:
                self.front_right_scan_angle = 12.0
            else:
                self.front_right_scan_value = min(front_right_scan_values)
            
            if min(front_scan_values) == np.inf:
                self.front_scan_value = 12.0
            else:
                self.front_scan_value = min(front_scan_values)

            if min(front_left_scan_values) == np.inf:
                self.front_left_scan_value = 12.0
            else:                
                self.front_left_scan_value = min(front_left_scan_values)
            
            if min(left_scan_values) == np.inf:
                self.left_scan_value = 12.0
            else:
                self.left_scan_value = min(left_scan_values)
                
    def main(self):
        print("main inited node running")
        print("Siguiente punto es:", self.target_postition_xy_2d)
        while not rospy.is_shutdown():
            if self.index < len(self.targets) -1:
                print("state is {s}".format(s = self.state))         
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
                        self.turn_left() 
                    elif self.state == "arrived":
                        print("ia llegue")
                        self.index += 1
                        self.target_postition_xy_2d = self.targets[self.index]
                        print("Siguiente punto es: {s}".format(s = self.target_postition_xy_2d))
                        self.state = "go_to_point"
                                     
                    self.vel_pub.publish(self.vel_msg)
            else: 
                print("ia llegue al final")  
            self.rate.sleep()          

if __name__ == "__main__":
    # TODO - Pass this as an argument
    targets = [(5.0, 5.0), (10.0, 0.0), (5.0, -5.0), (0.0, 0.0)]
    bug_0 = Bug0(targets,0.5)
    bug_0.main()
