#!/usr/bin/env python3


import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import time
import scipy
from messages.msg import Point, PointList


class Detector:

    def __init__(self):

        self.net = self.build_model(False)


        # self.left_pub = rospy.Publisher("/camera/left/point_list",PointList,queue_size=1)
        # self.right_pub = rospy.Publisher("/camera/right/point_list",PointList,queue_size=1)
        self.state_x_pub = rospy.Publisher("state_x",Float64,queue_size=1)
        self.state_y_pub = rospy.Publisher("state_y",Float64,queue_size=1)
        self.setpoint_x = rospy.Publisher("setpoint_x",Float64,queue_size=1)
        self.setpoint_y = rospy.Publisher("setpoint_y",Float64,queue_size=1)
        self.bridge = CvBridge()
        # self.rimage_sub = message_filters.Subscriber("/camera/right/image_raw",Image)
        self.limage_sub = message_filters.Subscriber("/camera/left/image_raw",Image).registerCallback(self.callback)
        # self.ts = message_filters.TimeSynchronizer([self.limage_sub, self.rimage_sub], 1).registerCallback(self.callback)

        self.INPUT_HEIGHT = 320
        self.INPUT_WIDTH = 320
        


    def callback(self,left):
        try:
            cv_image_left = CvBridge().imgmsg_to_cv2(left)
        except CvBridgeError as e:
            print(e)



        input = self.format_yolov5(cv_image_left)

        output = self.detect(input, self.net)

        class_ids, confidences, boxes = self.wrap_detection(input, output[0])



        for (classid, confidence, box) in zip(class_ids, confidences, boxes):
            color = (0, 255, 0)
            cv2.rectangle(cv_image_left, box, color, 2)
            cv2.rectangle(cv_image_left, (box[0], box[1]), (box[0] + box[2], box[1]+box[3]), color)
            cv2.putText(cv_image_left, "face", (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))

        cv2.imshow("images", cv_image_left)
        cv2.waitKey(1)

        # print(msg_left)


        if len(boxes) != 0:
            point = boxes[0]

            msg_state_x = Float64()
            msg_state_x.data = point[0] + (point[2]/2)

            msg_state_y = Float64()
            msg_state_y.data = point[1] + (point[3]/2)

            msg_setpoint_x = Float64()
            msg_setpoint_x.data = 320

            msg_setpoint_y = Float64()
            msg_setpoint_y.data = 240

            try:
                self.state_x_pub.publish(msg_state_x)
                self.state_y_pub.publish(msg_state_y)
                self.setpoint_x.publish(msg_setpoint_x)
                self.setpoint_y.publish(msg_setpoint_y)
            except Exception as e:
                print(e)



    def get_point_list(self, boxes):
        list = []
        for box in boxes:
            pt = Point()
            pt.X = box[0]
            pt.Y = box[1]
            list.append(pt)
        
        return list

    def format_yolov5(self,frame):
        frame = ((frame / np.max(frame)) * 255).astype('uint8')
        row, col, _ = frame.shape
        _max = max(col, row)
        result = np.zeros((_max, _max, 3), np.uint8)
        result[0:row, 0:col] = frame
        result.shape
        return result



    def detect(self,image, net):
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (self.INPUT_WIDTH, self.INPUT_HEIGHT), swapRB=True, crop=False)
        net.setInput(blob)
        preds = net.forward()
        return preds



    def wrap_detection(self,input_image, output_data):
        class_ids = []
        confidences = []
        boxes = []

        rows = output_data.shape[0]
        

        image_width, image_height, _ = input_image.shape

        x_factor = image_width / self.INPUT_WIDTH
        y_factor =  image_height / self.INPUT_HEIGHT

        for r in range(rows):
            row = output_data[r]

            confidence = row[4]
            if confidence >= 0.1:

                classes_scores = row[5:]
                _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
                class_id = max_indx[1]
                if (classes_scores[class_id] > .25):

                    confidences.append(confidence)

                    class_ids.append(class_id)

                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                    left = int((x - 0.5 * w) * x_factor)
                    top = int((y - 0.5 * h) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    box = np.array([left, top, width, height])
                    boxes.append(box)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45) 

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        return result_class_ids, result_confidences, result_boxes



    def build_model(self,is_cuda):
        # net = cv2.dnn.readNet('face_detection_yolov5s.onnx')
        net = cv2.dnn.readNet('/home/nakulj/Desktop/PeARL2023/Robot/src/eyes/weights/yolov5n-face.onnx')
        if is_cuda:
            print("Attempting to use CUDA")
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
        else:
            print("Running on CPU")
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

            
        return net
    


def main():
  rospy.init_node('face_detector', anonymous=True)
  detector = Detector()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
