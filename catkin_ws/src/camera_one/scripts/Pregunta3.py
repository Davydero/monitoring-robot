#! /usr/bin/python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 
import numpy as np

def main():
    rospy.init_node('image_sub')
    pub_sub = PubSub()
    pub_sub.start()
    rospy.spin()


class PubSub(object):

    def __init__(self):
        self.image = None
        self.bridge = CvBridge()
        self.loop_rate = rospy.Rate(2)
        self.pub = rospy.Publisher('/resultado', Image, queue_size=2)

        rospy.Subscriber("/usb_cam/image_raw/", Image, self.callback)

    def callback(self, msg):
        print('Recive an Image')
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.sleep(1)
        except e:
            print(e)
        else:
            rospy.loginfo('Found')
            im_gray=cv2.cvtColor(cv2_img,cv2.COLOR_BGR2GRAY)

            im_gray=np.float32(im_gray)
            dst=cv2.cornerHarris(im_gray,2,3,0.05)
            dst=cv2.dilate(dst,None)
            #cv2_img[dst>0.01 * dst.max()] = [255,0,255]
            cv2_img[dst>0.01 * dst.max()] = [255,0,255]

            self.image = cv2_img

    def start(self):
        while not rospy.is_shutdown():
            if self.image is not None:
                rospy.loginfo('Publicando Imagen')
                self.pub.publish(self.bridge.cv2_to_imgmsg(self.image))
            self.loop_rate.sleep()


if __name__ == '__main__':
    main()