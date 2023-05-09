import numpy as np
import cv2 as cv
img = cv.imread('book1_small.jpg')
# img = cv.imread('house_small.jpg')
gray= cv.cvtColor(img,cv.COLOR_BGR2GRAY)
####### SIFT
sift = cv.SIFT_create()
kp = sift.detect(gray,None)
kp, des = sift.detectAndCompute(gray, None)
####### FAST

# fast = cv.FastFeatureDetector_create()
# fast.setNonmaxSuppression(False)
# kp = fast.detect(gray, None)

####### ORB

# orb = cv.ORB_create(nfeatures=1000)
# kp, des = orb.detectAndCompute(gray, None)
####### 


img=cv.drawKeypoints(gray,kp,img)
# img=cv.drawKeypoints(gray,kp,img,flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


cv.imwrite('book1_keypoints.jpg',img)
# cv.imwrite('house_keypoints.jpg',img)