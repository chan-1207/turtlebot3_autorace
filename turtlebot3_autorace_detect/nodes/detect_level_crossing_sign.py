#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
import os
import cv2
from enum import Enum
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class DetectSign(Node):
    def __init__(self):
        super().__init__('detect_sign')

        self.sub_image_type = "raw" # you can choose image type "compressed", "raw"
        self.pub_image_type = "compressed" # you can choose image type "compressed", "raw"

        #subscribes
        if self.sub_image_type == "compressed":
            # subscribes compressed image
            self.sub_image_original = self.create_subscription(
                CompressedImage,
                '/detect/image_input/compressed',
                self.cbFindTrafficSign,
                10
            )
        elif self.sub_image_type == "raw":
            # subscribes raw image
            self.sub_image_original = self.create_subscription(
                Image,
                '/detect/image_input',
                self.cbFindTrafficSign,
                10
            )

        #publishes
        self.pub_traffic_sign = self.create_publisher(UInt8, '/detect/traffic_sign', 10)
        if self.pub_image_type == "compressed":
            # publishes traffic sign image in compressed type 
            self.pub_image_traffic_sign = self.create_publisher(
                CompressedImage,
                '/detect/image_output/compressed', 10
            )
        elif self.pub_image_type == "raw":
            # publishes traffic sign image in raw type
            self.pub_image_traffic_sign = self.create_publisher(
                Image, '/detect/image_output', 10
            )

        self.cvBridge = CvBridge()
        self.TrafficSign = Enum('TrafficSign', 'stop')
        self.counter = 1

        self.fnPreproc()

        self.get_logger().info("DetectSign Node Initialized")

    def fnPreproc(self):
        # Initiate SIFT detector
        self.sift = cv2.SIFT_create()

        dir_path = os.path.dirname(os.path.realpath(__file__))
        dir_path = dir_path.replace('turtlebot3_autorace_detect/nodes', 'turtlebot3_autorace_detect/')
        dir_path += 'image/'

        self.img_stop = cv2.imread(dir_path + 'stop.png',0)         # trainImage1
        self.kp_stop, self.des_stop = self.sift.detectAndCompute(self.img_stop,None)

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def fnCalcMSE(self, arr1, arr2):
            squared_diff = (arr1 - arr2) ** 2
            sum = np.sum(squared_diff)
            num_all = arr1.shape[0] * arr1.shape[1] #cv_image_input and 2 should have same shape
            err = sum / num_all
            return err

    def cbFindTrafficSign(self, image_msg):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        if self.sub_image_type == "compressed":
            #converting compressed image to opencv image
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        MIN_MATCH_COUNT = 4 #9
        MIN_MSE_DECISION = 50000

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(cv_image_input,None)

        matches_stop = self.flann.knnMatch(des1,self.des_stop,k=2)

        image_out_num = 1

        good_stop = []
        for m,n in matches_stop:
            if m.distance < 0.7*n.distance:
                good_stop.append(m)

        if len(good_stop)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good_stop ]).reshape(-1,1,2)
            dst_pts = np.float32([ self.kp_stop[m.trainIdx].pt for m in good_stop ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matches_stop = mask.ravel().tolist()

            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.stop.value

                self.pub_traffic_sign.publish(msg_sign)

                self.get_logger().info("stop")
                image_out_num = 2

        else:
            matches_stop = None
            # self.get_logger().info("nothing")

        if image_out_num == 1:
            if self.pub_image_type == "compressed":
                # publishes traffic sign image in compressed type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_input, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes traffic sign image in raw type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_imgmsg(cv_image_input, "bgr8"))

        elif image_out_num == 2:
            draw_params2 = dict(matchColor = (0,0,255), # draw matches in green color
                            singlePointColor = None,
                            matchesMask = matches_stop, # draw only inliers
                            flags = 2)

            final_stop = cv2.drawMatches(cv_image_input,kp1,self.img_stop,self.kp_stop,good_stop,None,**draw_params2)

            if self.pub_image_type == "compressed":
                # publishes traffic sign image in compressed type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(final_stop, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes traffic sign image in raw type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_imgmsg(final_stop, "bgr8"))             

def main(args=None):
    rclpy.init(args=args)
    node = DetectSign()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
