#!/usr/bin/env python

# Compares incoming intensity images with reference image
# and presents difference to inspect if mirror is rotating as desired
# based on number of non-zero pixels in difference

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

THRESH = 50
counts = []
ref_img = cv2.imread('ref.png', cv2.IMREAD_GRAYSCALE)
_, ref_img_binary = cv2.threshold(ref_img, THRESH, 255, cv2.THRESH_BINARY)

def image_callback(msg):
    # Convert ROS Image message to OpenCV image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

    # Convert the image to binary
    _, binary_image = cv2.threshold(cv_image, THRESH, 255, cv2.THRESH_BINARY)

    # Count the number of non-zero pixels
    count = np.count_nonzero(binary_image - ref_img_binary)
    counts.append(count)

    if len(counts) == 1000:
        plt.plot(range(1000), counts)
        plt.xlabel('Image #')
        plt.ylabel('Number of non-zero pixels')
        plt.yticks(np.arange(50, 150, 10))
        plt.show()
        cv2.imwrite('diff.png', binary_image - ref_img_binary)
        cv2.imwrite('cur.png', binary_image)

    rospy.loginfo("Number of non-zero pixels: %d", count)

def image_processing_node():
    rospy.init_node('image_processing_node', anonymous=True)

    # Subscribe to the image topic
    rospy.Subscriber('/blueversion/ir_image/all', Image, image_callback)

    # Spin until node is shutdown
    rospy.spin()

if __name__ == '__main__':
    try:
        image_processing_node()
    except rospy.ROSInterruptException:
        pass

