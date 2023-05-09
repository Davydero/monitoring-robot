#! /usr/bin/python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bridge = CvBridge()

a = 0
def image_callback(msg):
    global a
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        rospy.sleep(1)
    except e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
         kernel1 = np.array([[1, 2, 1.1],
                   [1, -1.3, -0.1],
                   [-0.8, -2, -1.4]])

         kernel2 = np.array([[-0.8, -2, -1.4],
                   [1, -1.3, -0.1],
                   [1, 2, 1.1]])
         kernel3 = np.array([[-0.8, 1, 1],
                   [-2, -1.3, 2],
                   [-1.4, -0.1, 1.1]])
         kernel4 = np.array([[1, 1, -0.8],
                   [2, -1.3, -2],
                   [1.1, -0.1, -1.4]])
         
         #dst = cv2.filter2D(cv2_img, -1, kernel)
         dst1 = cv2.filter2D(cv2_img, -1, kernel1)
         dst2 = cv2.filter2D(cv2_img, -1, kernel2)
         dst3 = cv2.filter2D(cv2_img, -1, kernel3)
         dst4 = cv2.filter2D(cv2_img, -1, kernel4)
         dst = dst1+dst2+dst3+dst4

         cv2.imwrite('camera_image_{}.png'.format(a), dst)
         a = a+1


def main():
    rospy.init_node('image_sub')
    image_topic = "/usb_cam/image_raw/"

    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()


if __name__ == '__main__':
    main()