#! /usr/bin/python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


def main():
    pub = rospy.Publisher('/resultado', Image, queue_size=2)
    rospy.init_node('image_sub')
    loop_rate = rospy.Rate(1)
    while not rospy.is_shutdown():
            cv2_img = cv2.imread('partido_futbol.png')
            imgAug = cv2_img.copy()
            im_tar = cv2.imread('monopoly.png')
            im_tar2 = cv2.imread('python-logo.png')
            rospy.sleep(1)
            bridge = CvBridge()

            rospy.loginfo("Found")
            #cv2.drawChessboardCorners(cv2_img, (8, 6), corners, found)
            im_src = cv2_img

            #pts_dst = np.array([ [467,255],[710,193],[470,290],[710,224]]).reshape(-1,1,2)
            #pts_src = np.array([ [0, 0],[250, 0],[0, 250],[250, 250]]).reshape(-1,1,2)
            pts_dst = np.array([ [467,255],[710,193],[710,224],[470,290]]).reshape(-1,1,2)#poner los pixeles que se quieren reemplazar en la imagen principal
            #pts_src = np.array([ [0, 0],[250, 0],[250, 250],[0, 250]]).reshape(-1,1,2)
            pts_src = np.array([ [0, 0],[410, 0],[410, 123],[0, 123]]).reshape(-1,1,2)#poner coordenadas de la imagen a montar
            h, status = cv2.findHomography(pts_src, pts_dst)

            pts = np.float32([ [467,255],[710,193],[710,224],[470,290]]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts, h)
            img2 = cv2.polylines(im_src,[np.int32(pts_dst)],True,(255,0,255),3)
            im_out = cv2.warpPerspective(im_tar, h, (img2.shape[1],img2.shape[0]))
            maskNew = np.zeros((cv2_img.shape[0],cv2_img.shape[1]),np.uint8)
            cv2.fillPoly(maskNew,[np.int32(pts_dst)], (255,255,255))
            maskInv = cv2.bitwise_not(maskNew)
            imgAug = cv2.bitwise_and(imgAug, imgAug, mask = maskInv)
            imgAug = cv2.bitwise_or(im_out, imgAug)



            pts_dst = np.array([ [220,246],[443,190],[443,307],[224,370]]).reshape(-1,1,2)#poner los pixeles que se quieren reemplazar en la imagen principal
            #pts_src = np.array([ [0, 0],[250, 0],[250, 250],[0, 250]]).reshape(-1,1,2)
            pts_src = np.array([ [0, 0],[1333, 0],[1333, 800],[0, 800]]).reshape(-1,1,2)#poner coordenadas *(dimensiones) de la imagen a montar
            h, status = cv2.findHomography(pts_src, pts_dst)

            pts = np.float32([ [220,246],[443,190],[443,307],[224,370]]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts, h)
            img2 = cv2.polylines(imgAug,[np.int32(pts_dst)],True,(255,0,255),3)
            im_out = cv2.warpPerspective(im_tar2, h, (img2.shape[1],img2.shape[0]))
            maskNew = np.zeros((cv2_img.shape[0],cv2_img.shape[1]),np.uint8)
            cv2.fillPoly(maskNew,[np.int32(pts_dst)], (255,255,255))
            maskInv = cv2.bitwise_not(maskNew)
            imgAug = cv2.bitwise_and(imgAug, imgAug, mask = maskInv)
            imgAug = cv2.bitwise_or(im_out, imgAug)




            image = imgAug




            if image is not None:
                rospy.loginfo('Publicando Imagen')
                pub.publish(bridge.cv2_to_imgmsg(image))
                loop_rate.sleep()




if __name__ == '__main__':
    main()