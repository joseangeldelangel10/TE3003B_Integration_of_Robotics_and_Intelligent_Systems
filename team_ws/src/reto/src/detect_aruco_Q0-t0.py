#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Int8, Header


class ArucoDetector():
    def __init__(self, aruco_dict = cv2.aruco.DICT_4X4_50):

        rospy.init_node("aruco_detector")

        # ________ aruco atributes initialization ______
        self.arucoDict = cv2.aruco.getPredefinedDictionary(aruco_dict)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.arucoDetector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)

        # ________ ros atributes initialization ______        
        self.image_pub = rospy.Publisher("/image_detecting", Image, queue_size = 1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_calback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan,self.scan_callback)

        #__________ image ______________
        self.curr_signs_image_msg = Image()
        self.curr_signs_image_msg_2 = Image()
        self.curr_signs_image_msg_3 = Image()

        #___________ color ______________
        self.green = (0, 255, 0)

        #___________ video initialization _______________
        self.vid = cv2.VideoCapture(0)
        self.displayed_image_ocv = np.zeros(5, dtype=np.uint8)
        

    def scan_callback(self, msg):
        pass

    def odom_calback(self, msg):
        pass


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

    def midpoint_equation(self, p1, p2):
        return ( (p1[0]+p2[0])/2, (p1[1]+p2[1])/2 )

    def get_aruco_midpoint(self, rectangle_corners):
        """ function that returns the x,y,z cordinates of the aruco's midpoint """
        # ______________ initializing and formating data that will be used ______________
        self.arucos_mask = np.zeros((self.image_size.height, self.image_size.width, 3), dtype = np.uint8)
        rectangle_corners_for_x_y = rectangle_corners.reshape((4,2))
        rectangle_corners_for_mask = np.int32(rectangle_corners.reshape((1,4,2)))
        # ______________ getting the x,y cordinates of the aruco tag (in pixels) ______________
        x_center, y_center = self.midpoint_equation(rectangle_corners_for_x_y[0,:], rectangle_corners_for_x_y[2,:])
        # ______________ getting the z cordinate of the aruco tag (in point cloud units) ______________
        # step one - we filter the point cloud using a mask with only the area of the aruco tag
        cv2.fillPoly(self.arucos_mask, pts = rectangle_corners_for_mask, color=(255,255,255))  
        one_channel_arucos_mask = cv2.cvtColor(self.arucos_mask, cv2.COLOR_BGR2GRAY)  /255.0        
        self.arucos_mask_with_distance = np.nan_to_num(self.point_cloud_ocv)*one_channel_arucos_mask
        # step two - we get the mean point cloud value on the aruco tag area, to use it as z value
        tag_area = one_channel_arucos_mask.sum()
        if tag_area > 0:        
            z_center = (self.arucos_mask_with_distance/255.0).sum()/tag_area
        else:
            z_center = 0.0        
        # on the next line the minus signs allow us to transform the camera reference frame to a reference frame that
        # is parallel to the robots reference frame 
        return (float(z_center), float(-x_center), float(-y_center)) 

    def transform_aruco_midpoint_to_metric_system(self, aruco_midpoint): 
   
        x_m = (0.25738586)*(aruco_midpoint[0]) + 0.05862189

        x_px_times_y_px = aruco_midpoint[0]*aruco_midpoint[1]
        y_m = 0.29283879*aruco_midpoint[0] + 0.00050015*aruco_midpoint[1] + 0.00094536*x_px_times_y_px + 0.23096646

        x_px_times_z_px = aruco_midpoint[0]*aruco_midpoint[2]
        z_m = 0.16725805*aruco_midpoint[0] - 0.00069012*aruco_midpoint[2] + 0.00098029*x_px_times_z_px - 0.04520938         
        return (x_m, y_m, z_m)

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
            
    def euclidean_distance(self, tuple):
        return math.sqrt(tuple[0]**2 + tuple[1]**2 + tuple[2]**2)

    def main(self):
        while not rospy.is_shutdown():
            
            _, self.frame = self.vid.read()

            aruco_corners, aruco_ids = self.get_arucos_info_in_image(self.frame)
            print (aruco_ids)
            self.displayed_image_ocv = self.frame.copy()
            if len(aruco_corners) > 0:
                self.displayed_image_ocv = self.draw_arucos(self.displayed_image_ocv, aruco_corners)                
                
                #aruco_centers = list(map(self.get_aruco_midpoint, aruco_corners))

                #centers_meters = list(map(self.transform_aruco_midpoint_to_metric_system, aruco_centers))           
                #closest_aruco_position = self.get_closest_point(centers_meters)

                #closest_aruco_position = self.transform_aruco_midpoint_to_metric_system(closest_aruco_position)                

            self.curr_signs_image_msg = self.cv2_to_imgmsg(self.displayed_image_ocv, encoding = "bgr8")
            self.image_pub.publish(self.curr_signs_image_msg)
        self.vid.release()
        cv2.destroyAllWindows()    

if __name__ == "__main__":
    aruco_detector = ArucoDetector()
    aruco_detector.main()