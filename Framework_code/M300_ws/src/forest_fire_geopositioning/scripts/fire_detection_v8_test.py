#!/usr/bin/env python3

import cv2

# check python version
import sys
print("Python version: " + sys.version)

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


from sensor_msgs.msg import Image as RosImage
from vision_msgs.msg import Detection2D, Detection2DArray
from cv_bridge import CvBridge, CvBridgeError





# Check Python version
print("Python version: " + sys.version)

# Initialize paths and model
get_path = pathlib.Path.cwd()
date = datetime.datetime.now().strftime("%Y%m%d")

classNames = ["Wildfire Spot"]
model = YOLO("/home/uav/Downloads/YoloWeights/best.pt")
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

# Initialize ROS node
rospy.init_node('image_processor', anonymous=True)
pub = rospy.Publisher('/bounding_boxes/fire_spots', Detection2DArray, queue_size=10)
cv_bridge = CvBridge()

# Path to the image file
image_path = '/home/uav/M300_ws/src/forest_fire_geopositioning/scripts/Wide_sample.jpg'  # Replace with the path to your image

def process_image_and_publish():
    try:
        # Read the image
        frame = cv2.imread(image_path)
        if frame is None:
            print(f"Failed to load image from {image_path}")
            return

        # Process the image with YOLO
        results = model(frame, stream=True)
        ros_boxes = Detection2DArray()
        ros_boxes.header.stamp = rospy.Time.now()  # Use current time for the header

        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 1)

                # Add results to the Detection2DArray
                ros_box = Detection2D()
                ros_box.header.stamp = ros_boxes.header.stamp
                ros_box.bbox.size_x = x2 - x1
                ros_box.bbox.size_y = y2 - y1
                ros_box.bbox.center.x = (x1 + x2) / 2
                ros_box.bbox.center.y = (y1 + y2) / 2
                ros_box.bbox.center.theta = 0
                ros_boxes.detections.append(ros_box)

        # Publish the bounding boxes
        pub.publish(ros_boxes)
        print("======> Number of boxes detected: ", len(ros_boxes.detections))

        # Optionally display the processed image
        # cv2.imshow('Processed Image', frame)
        # cv2.waitKey(1)

    except CvBridgeError as e:
        print(e)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            process_image_and_publish()
            rospy.sleep(1)  # Adjust the sleep duration as needed
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

