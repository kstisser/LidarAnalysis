#!/usr/bin/env python
from time import sleep
import numpy as np
import rospy
from std_msgs.msg import Float32
import cv2
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class CB:
    def __init__(self):
        self.pub = rospy.Publisher("/histogram", Image, queue_size=10)

    def callback(self, image_message):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
        
        #for row in cv_image
        for i in range(398,415):
           for j in range(288,306):
              print(cv_image[i,j])
        #print(cv_image[398,288])
        #print(cv_image[415,306])

        #histogram = cv2.calcHist([cv_image], [0], None, [256], [0.0, 1.0])
        #hist = np.bincount(cv_image.ravel(),minlength=256)
        #plt.plot(histogram, color='k')
        #plt.show()
        #try:
        #    self.pub.publish(bridge.cv2_to_imgmsg(cv_image, "16UC1"))
        #except CvBridgeError as e:
        #    print(e)

if __name__ == '__main__':
    try:
        sr = CB()
        rospy.init_node('histogram', anonymous=True)
        rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data", Image, sr.callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
