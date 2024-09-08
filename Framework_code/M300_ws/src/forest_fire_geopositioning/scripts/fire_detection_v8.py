#!/usr/bin/env python3

import cv2

# check python version
import sys
import subprocess
print("Python version:", sys.version)
print("Python path:", sys.path)
# print("Installed packages:")
# subprocess.run([sys.executable, '-m', 'pip', 'list'])

import torch
import glob
import os
from numpy import asarray
import PIL.Image as Image
# import pandas
import csv
from ultralytics import YOLO

import time
import math
import pathlib
import datetime

import rospy

# callback function to show the image using opencv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray

get_path = pathlib.Path.cwd()
date = datetime.datetime.now().strftime("%Y%m%d")

# classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
#               "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
#               "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
#               "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
#               "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
#               "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
#               "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
#               "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
#               "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
#               "teddy bear", "hair drier", "toothbrush"
#               ]
# model = YOLO("YoloWeights/yolov8n.pt")

# to get the parameter:
# yolo task=detect mode=train model=yolov8n.pt data=AVITAGS_NAVLAB20230930-1/data.yaml epochs=30 imgsz=640
classNames = ["Wildfire Spot"]
model = YOLO("/home/erfan/Downloads/YoloWeights/best.pt")
# model = YOLO("/home/qin/m300_ws/src/forest_fire_fighting/scripts/YoloWeights/yolov8n.pt")

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
# params for recording the yolo processed video
# frame_size = (640, 480)  # width, height
# fourcc = cv2.VideoWriter_fourcc(*"mp4v")
# print("[INFO]", date)
# save_path = 'home/nav/dev/datasets/videos/20231016/'

# import vision_msgs
from vision_msgs.msg import Detection2D, Detection2DArray

# a dash line to draw in the middle of the wide image
def draw_dashed_line(image, start_point, end_point, color, thickness, dash_length=10):
    line_type = 8  # line type for dashed line
    for i in range(start_point[1], end_point[1], dash_length * 2):
        cv2.line(image, (start_point[0], i), (start_point[0], i + dash_length), color, thickness, line_type)

def callback(image, pub):
    try:
        frame = CvBridge().imgmsg_to_cv2(image, "bgr8")
        
        # Print the size of the image
        height, width, channels = frame.shape
        print(f"Image size: width={width}, height={height}, channels={channels}")

        # Draw a dashed line in the middle of the image
        middle_x = width // 2
        draw_dashed_line(frame, (middle_x, 0), (middle_x, height), (0, 255, 0), 2)

        # Process frame with YOLO model
        results = model.track(frame, stream=True, conf=0.3)
        ros_boxes = Detection2DArray()
        ros_boxes.header = image.header
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 1)
                
                confidence = math.ceil((box.conf[0] * 100)) / 100
                print("======> Confidence", confidence)
                
                # Calculate and draw the center of the bounding box
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                cv2.circle(frame, (center_x, center_y), 3, (0, 0, 255), -1)  # Visualize the center

                # Add results to the Detection2DArray
                ros_box = Detection2D()
                ros_box.header = image.header
                ros_box.bbox.size_x = x2 - x1
                ros_box.bbox.size_y = y2 - y1
                ros_box.bbox.center.x = center_x
                ros_box.bbox.center.y = center_y
                ros_box.bbox.center.theta = 0
                ros_boxes.detections.append(ros_box)
        

            pub.publish(ros_boxes)

        # Show the image with bounding boxes, centers, and the middle dashed line
        cv2.imshow('processed', frame)
        cv2.waitKey(1)

    except CvBridgeError as e:
        print(e)

def listener():
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('/bounding_boxes/fire_spots', Detection2DArray, queue_size=10)
    rospy.Subscriber("/dji_osdk_ros/main_wide_RGB", Image, callback, pub)
    rospy.spin()

if __name__ == '__main__':
    listener()
    cv2.destroyAllWindows()
