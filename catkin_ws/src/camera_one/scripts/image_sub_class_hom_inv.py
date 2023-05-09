#! /usr/bin/python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

im_tar = cv2.imread('logoRos.jpeg')

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
        self.pub = rospy.Publisher('/resultado', Image, queue_size=2)

        rospy.Subscriber("/usb_cam/image_raw/", Image, self.callback)

    def callback(self, msg):
        print('imagen recibida')
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            imgAug = cv2_img.copy()
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
                pts_dst = np.array([ corners[0,0],corners[40,0],corners[47,0],corners[7,0]]).reshape(-1,1,2)
                pts_src = np.array([ [0, 0],[0, 250],[250, 250],[250, 0]]).reshape(-1,1,2)
                h, status = cv2.findHomography(pts_src, pts_dst)

                pts = np.float32([[0,0],[0,250],[250,250],[250,0]]).reshape(-1,1,2)
                dst = cv2.perspectiveTransform(pts, h)
                img2 = cv2.polylines(im_src,[np.int32(pts_dst)],True,(255,0,255),3)

                ##im_out = cv2.warpPerspective(im_tar, h, (250,250))
                # im_out = cv2.warpPerspective(im_tar, h, (250,250))
                im_out = cv2.warpPerspective(im_tar, h, (img2.shape[1],img2.shape[0]))
                maskNew = np.zeros((cv2_img.shape[0],cv2_img.shape[1]),np.uint8)
                cv2.fillPoly(maskNew,[np.int32(pts_dst)], (255,255,255))
                maskInv = cv2.bitwise_not(maskNew)
                imgAug = cv2.bitwise_and(imgAug, imgAug, mask = maskInv)
                imgAug = cv2.bitwise_or(im_out, imgAug)


                self.image = imgAug
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