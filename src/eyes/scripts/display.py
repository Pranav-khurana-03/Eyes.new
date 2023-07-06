#!/usr/bin/env python3

import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import yaml
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import time
import scipy
import rospy


class Display:

    def __init__(self, capNum, cameraName, metadataPath):
        self.imagePub = rospy.Publisher(cameraName + "/image_raw",Image,queue_size=1)

        self.infoPub = rospy.Publisher(cameraName + "/camera_info",CameraInfo,queue_size=1)

        self.metadataPath = metadataPath

        self.capture = cv2.VideoCapture(capNum)

        self.bridge = CvBridge()

        self.cameraName = cameraName

        print("Up and Running")
    
    def publish(self):

        try:
          ret, frame = self.capture.read()

          self.imagePub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

          camera_info_msg = yaml_to_CameraInfo(self.metadataPath)

          self.infoPub.publish(camera_info_msg)


          # print(frameL)

          # img = cv2.hconcat([frameL, frameR])
        #   cv2.imshow("image", frame)
        #   cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

def yaml_to_CameraInfo(yaml_fname):
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.safe_load(file_handle)


    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = "camera"
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg



