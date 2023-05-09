import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

img1 = cv.imread('waldo.jpg',cv.IMREAD_GRAYSCALE)  # queryImage im_en
img2 = cv.imread('where-is-waldo.jpg',cv.IMREAD_GRAYSCALE) # trainImage
hE, wE = img1.shape

sift = cv.SIFT_create()
# find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(img1,None)
kp2, des2 = sift.detectAndCompute(img2,None)

bf = cv.BFMatcher()
matches = bf.knnMatch(des1,des2, k=2)
good = []
for m,n in matches:
    if m.distance <0.75 *n.distance:
        good.append(m)
print(len(good))
                
if len(good) > 100:
    imgFeatures = cv.drawMatches(img1,kp1,img2,kp2,good,None,flags=2)
    pts_src = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1,1,2)
    pts_dst = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1,1,2)
    h, status = cv.findHomography(pts_src, pts_dst)

    img1 = img1.reshape(-1,1,2)
    pts = np.float32([[0,0],[0,hE],[wE,hE],[wE,0]]).reshape(-1,1,2)
    dst = cv.perspectiveTransform(pts, h)

    img2 = cv.polylines(img2,[np.int32(dst)],True,(0,255,0),10)
plt.imshow(imgFeatures),plt.show()
plt.imshow(img2),plt.show()