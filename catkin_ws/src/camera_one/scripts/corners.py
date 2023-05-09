import cv2 as cv
import numpy as np
import random as random
output_window = 'Harris'
harris_threshold = 50
max_threshold = 100
random.seed(12345)
def harris_corners(val):
    myHarris_copy = np.copy(src)
    harris_threshold = max(val, 1)
    for i in range(src_gray.shape[0]):
        for j in range(src_gray.shape[1]):
            if cov_m[i,j] > myHarris_minVal + ( myHarris_maxVal - myHarris_minVal )*harris_threshold/max_threshold:
                cv.circle(myHarris_copy, (j,i), 4, (random.randint(0,256), random.randint(0,256), random.randint(0,256)), cv.FILLED)
    cv.imshow(output_window, myHarris_copy)


src = cv.imread("book1_small.jpg")
if src is None:
    print('Could not open or find the image:', "book1_small.jpg")
    exit(0)
src_gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
# Parameters
blockSize = 3
apertureSize = 3
# Eigenvals Eigenvecs
harris_eigvalvec = cv.cornerEigenValsAndVecs(src_gray, blockSize, apertureSize)
# calculate cov_m
cov_m = np.empty(src_gray.shape, dtype=np.float32)
for i in range(src_gray.shape[0]):
    for j in range(src_gray.shape[1]):
        lambda_1 = harris_eigvalvec[i,j,0]
        lambda_2 = harris_eigvalvec[i,j,1]
        cov_m[i,j] = lambda_1*lambda_2 - 0.04*pow( ( lambda_1 + lambda_2 ), 2 )
myHarris_minVal, myHarris_maxVal, _, _ = cv.minMaxLoc(cov_m)

cv.namedWindow(output_window)
cv.createTrackbar('Threshold:', output_window, harris_threshold, max_threshold, harris_corners)
harris_corners(harris_threshold)

cv.waitKey()