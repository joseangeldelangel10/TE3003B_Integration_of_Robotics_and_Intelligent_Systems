#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header, Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from nav_functions import *


class ArucoDetector():
    def __init__(self, aruco_dict = cv2.aruco.DICT_4X4_50):

        rospy.init_node("aruco_detector")

        # ________ aruco atributes initialization ______
        self.arucoDict = cv2.aruco.getPredefinedDictionary(aruco_dict)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.arucoDetector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)
        self.arucoCoordinates = {"0": (2.0,2.0),"1": (2.0,-2.0),"2": (-2.0,-2.0),"3": (-2.0,2.0),"4": (4.0,4.0),"5": (4.0,-4.0),"6": (-4.0,-4.0),"7": (-4.0,4.0)}
        #self.arucoBoxDim = 0.24

        # ________ ros atributes initialization ______        
        self.image_pub = rospy.Publisher("/image_detecting", Image, queue_size = 1)
        self.visual_sensor_reading_pub = rospy.Publisher("/visual_sensor_reading", Float64MultiArray, queue_size = 1)        
        self.visual_sensor_reading_msg = Float64MultiArray()
        self.relation_matrix_between_sensor_and_state_pub = rospy.Publisher("/relation_matrix_between_sensor_and_state", Float64MultiArray, queue_size = 1)        
        self.relation_matrix_between_sensor_and_state_msg = Float64MultiArray()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan,self.scan_callback)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

        #__________ image ______________        
        self.curr_signs_image_msg = Image()        

        #___________ color ______________
        self.green = (0, 255, 0)

        #___________ video initialization _______________
        self.displayed_image_ocv = np.zeros(5, dtype=np.uint8)
        self.image = None
        self.ocv_image = None

        self.scan = None

        self.current_position_xy_2d = None
        self.current_angle = None        

        self.rate = rospy.Rate(5.0)        

    def image_callback(self, msg):
        self.image = msg

    def scan_callback(self, msg):
        self.scan = msg

    def odom_callback(self, data):     
        self.bypass_odom = data   
        self.current_position_xy_2d = ( data.pose.pose.position.x , data.pose.pose.position.y )                             
        _, _, yaw = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        self.current_angle = yaw
        #print("current angle {c}".format(c = self.current_angle))

    def fill_sensor_data_multi_array_layout(self):
        layout = MultiArrayLayout()
        dimension_0 = MultiArrayDimension()
        dimension_0.label = "len"
        dimension_0.size = 2
        dimension_0.stride = 2

        layout.dim[0] = dimension_0       
        layout.data_offset = 0
        
        self.visual_sensor_reading_msg.layout = layout

    def fill_relation_matrix_between_sensor_and_state_multi_array_layout(self):
        layout = MultiArrayLayout()
        
        dimension_0 = MultiArrayDimension()
        dimension_0.label = "height"
        dimension_0.size = 2
        dimension_0.stride = 2*3

        dimension_1 = MultiArrayDimension()
        dimension_1.label = "width"
        dimension_1.size = 3
        dimension_1.stride = 3

        layout.dim[0] = dimension_0
        layout.dim[1] = dimension_1        
        layout.data_offset = 0
        
        self.relation_matrix_between_sensor_and_state_msg.layout = layout

    def draw_arucos(self, image, corners):
        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
			# loop over the detected ArUCo corners
            for markerCorner in corners:
				# extract the marker corners (which are always returned
				# in top-left, top-right, bottom-right, and bottom-left
				# order)
                corners = markerCorner.reshape((4, 2))

                (topLeft, topRight, bottomRight, bottomLeft) = corners

				# convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

				# draw the bounding box of the ArUCo detection
                image = cv2.line(image, topLeft, topRight, self.green, 2)
                image = cv2.line(image, topRight, bottomRight, self.green, 2)
                image = cv2.line(image, bottomRight, bottomLeft, self.green, 2)
                image = cv2.line(image, bottomLeft, topLeft, self.green, 2)
        return image

    def get_arucos_info_in_image(self, image):
        # detect ArUco markers in the input frame
        (corners, ids, rejected) = self.arucoDetector.detectMarkers(image)    
        return (corners, ids)

    def cv2_to_imgmsg(self, image, encoding = "bgr8"):
        #print("cv2_to_imgmsg image shape is:" + str(image.shape))
        if encoding == "bgr8":
            self.curr_signs_image_msg.header = Header()
            self.curr_signs_image_msg.height = image.shape[0]
            self.curr_signs_image_msg.width = image.shape[1]
            self.curr_signs_image_msg.encoding = encoding
            self.curr_signs_image_msg.is_bigendian = 0
            self.curr_signs_image_msg.step = image.shape[1]*image.shape[2]

            data = np.reshape(image, (self.curr_signs_image_msg.height, self.curr_signs_image_msg.step) )
            data = np.reshape(image, (self.curr_signs_image_msg.height*self.curr_signs_image_msg.step) )
            data = list(data)
            self.curr_signs_image_msg.data = data
            return self.curr_signs_image_msg
        else:            
            raise Exception("Error while convering cv image to ros message") 
    
    def imgmsg_to_cv2(self, ros_image):
        if (ros_image.encoding == "bgr8" or ros_image.encoding == "rgb8") and ros_image.is_bigendian == 0:
            cv_img = np.array(list(ros_image.data), dtype= np.uint8)
            cv_img = np.reshape(cv_img, (ros_image.height, ros_image.step))
            cv_img = np.reshape(cv_img, (ros_image.height, ros_image.width, int(ros_image.step/ros_image.width) ) )
        else:
            cv_img = None
            raise Exception("Error while convering ros image message to a cv image")                  
        return cv_img
    
    def euclidean_distance(self, tuple):
        return math.sqrt(tuple[0]**2 + tuple[1]**2)

    def id2coordinate(self, id):
        aruco_coordinate = self.arucoCoordinates[str(id)]
        return aruco_coordinate
        
    def get_aruco_area_given_corners(self, unshaped_corners):
        corners = unshaped_corners.reshape((4,2))
        corners_for_det = np.concatenate((corners, corners[0,:].reshape((1,2))), axis = 0)
        det1 = np.linalg.det( corners_for_det[:2,:] )
        det2 = np.linalg.det( corners_for_det[1:3,:] )
        det3 = np.linalg.det( corners_for_det[2:4,:] )
        det4 = np.linalg.det( corners_for_det[3:5,:] )
        area = (det1 + det2 + det3 + det4)/2.0
        return area
    
    def filter_to_only_biggest_area_aruco(self, aruco_corners, aruco_ids):        
        aruco_areas = list(map(self.get_aruco_area_given_corners, aruco_corners))
        aruco_corners_areas_and_ids = list(zip(aruco_areas, aruco_corners, aruco_ids))
        aruco_corners_areas_and_ids.sort(reverse = True, key = lambda x:x[0])
        return (aruco_corners_areas_and_ids[0][1], aruco_corners_areas_and_ids[0][2])

    def publish_sensor_data(self, aruco_cordinates):
        if self.current_position_xy_2d != None and self.current_angle != None:
            delta_x = aruco_cordinates[0] - self.current_position_xy_2d[0]
            delta_y = aruco_cordinates[1] - self.current_position_xy_2d[1]
            p = self.euclidean_distance((delta_x, delta_y))
            alpha = np.arctan2(delta_y, delta_x) - self.current_angle
            
            relation_matrix_between_sensor_and_state = np.array(
                [[ -(delta_x/p), -(delta_y/p), 0.0],
                 [ (delta_y/(p**2)), -(delta_x/(p**2)), -1.0 ]]
            )
            flattened_relation_matrix_between_sensor_and_state = relation_matrix_between_sensor_and_state.reshape((6, ))
            
            self.relation_matrix_between_sensor_and_state_msg.data = flattened_relation_matrix_between_sensor_and_state.tolist()
            self.relation_matrix_between_sensor_and_state_pub.publish(self.relation_matrix_between_sensor_and_state_msg)
            self.visual_sensor_reading_msg.data = [p, alpha]
            self.visual_sensor_reading_pub.publish(self.visual_sensor_reading_msg)

    def main(self):
        while not rospy.is_shutdown():            
            if self.image != None and self.scan != None and self.current_position_xy_2d != None:                
                self.ocv_image = self.imgmsg_to_cv2(self.image)
                arucos_corners, arucos_ids = self.get_arucos_info_in_image(self.ocv_image)
                self.displayed_image_ocv = self.ocv_image.copy()
                
                if len(arucos_corners) > 0:
                    aruco_corners, aruco_id = self.filter_to_only_biggest_area_aruco(arucos_corners, arucos_ids)
                    arucos_corners = [aruco_corners]
                    self.displayed_image_ocv = self.draw_arucos(self.displayed_image_ocv, arucos_corners)
                    self.displayed_image_ocv = cv2.resize(self.displayed_image_ocv, (100,100), interpolation = cv2.INTER_AREA)

                    aruco_cordinates = self.id2coordinate(aruco_id[0])
                    self.publish_sensor_data(aruco_cordinates)                        

                self.curr_signs_image_msg = self.cv2_to_imgmsg(self.displayed_image_ocv, encoding = "bgr8")
                self.image_pub.publish(self.curr_signs_image_msg)
                                
            self.rate.sleep()
   

if __name__ == "__main__":
    aruco_detector = ArucoDetector()
    aruco_detector.main()