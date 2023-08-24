#!/usr/bin/env python

# Useful for analyzing how power fluctuates over time.
# Used together with lcw_analyze_imgs.py

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import message_filters
from pathlib import Path

THRESH = 50
ir_images = []
raw_images = []
count = 0


def image_callback(msg1, msg2):
    # Convert ROS image message to OpenCV image
    global count
    count+=1
    print(count)
    if count%40 == 0:
        bridge = CvBridge()
        ir_image = bridge.imgmsg_to_cv2(msg1, desired_encoding='mono8')
        raw_image = bridge.imgmsg_to_cv2(msg2, desired_encoding='mono8')
        ir_images.append(ir_image)
        raw_images.append(raw_image)

        if count == 64000:
            # save images
            for i in range(len(raw_images)):
                cv2.imwrite('images/raw_' + str(i) + '.png', raw_images[i])
                cv2.imwrite('images/ir_' + str(i) + '.png', ir_images[i])

def image_processing_node():
    rospy.init_node('image_processing_node', anonymous=True)

    # rospy.Subscriber('/blueversion/ir_image/all', Image, image_callback)

    image1_sub = message_filters.Subscriber('/blueversion/ir_image/all', Image)
    image2_sub = message_filters.Subscriber('/blueversion/raw_image/all', Image)

    ts = message_filters.ApproximateTimeSynchronizer([image1_sub, image2_sub], queue_size=10, slop=0.1)
    ts.registerCallback(image_callback)

    # create images directory if it doesn't exist using Pathlib
    Path("images").mkdir(parents=True, exist_ok=True)

    # Spin until node is shutdown
    rospy.spin()

if __name__ == '__main__':
    try:
        image_processing_node()
    except rospy.ROSInterruptException:
        pass

