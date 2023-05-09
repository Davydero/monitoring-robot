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
        self.loop_rate = rospy.Rate(1)
        self.pub = rospy.Publisher('/resultado', Image, queue_size=3)

        rospy.Subscriber("/usb_cam/image_raw/", Image, self.callback)

    def callback(self, msg):
        print('imagen recibida')
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.sleep(1)
        except e:
            print(e)
        else:
            found, corners = cv2.findChessboardCorners(
                cv2_img, (8, 6), flags=cv2.CALIB_CB_FAST_CHECK)
            if found:
                rospy.loginfo("Found")
                #cv2.drawChessboardCorners(cv2_img, (8, 6), corners, found)
                im_src = cv2_img
                pts_src = np.array([ corners[0,0],corners[40,0],corners[47,0],corners[7,0]])
                pts_dst = np.array([ [0, 0],[0, 500],[500, 500],[500, 0]])
                h, status = cv2.findHomography(pts_src, pts_dst)
                im_out = cv2.warpPerspective(im_src, h, (600,600))
                self.image = im_out
                print('esquinas')
                print(corners[0,0])
                print(corners[7,0])
                print(corners[40,0])
                print(corners[47,0])


    def start(self):
        while not rospy.is_shutdown():
            if self.image is not None:
                rospy.loginfo('Publicando Imagen')
                self.pub.publish(self.bridge.cv2_to_imgmsg(self.image))
            self.loop_rate.sleep()


if __name__ == '__main__':
    main()