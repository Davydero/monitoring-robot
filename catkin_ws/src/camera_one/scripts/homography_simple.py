#!/usr/bin/env python

import cv2
import numpy as np

if __name__ == '__main__' :
    im_src = cv2.imread('road.jpeg')
    pts_src = np.array([ [360, 326],[498, 326],[586, 374],[2, 374]])
    im_dst = cv2.imread('road.jpeg')
    pts_dst = np.array([ [0, 0],[0, 500],[500, 500],[500, 0]])
    h, status = cv2.findHomography(pts_src, pts_dst)

    #im_out = cv2.warpPerspective(im_src, h, (im_dst.shape[1],im_dst.shape[0]))
    im_out = cv2.warpPerspective(im_src, h, (500,500))

    scale_percent = 60 # percent of original size
    width = int(im_src.shape[1] * scale_percent / 100)
    height = int(im_src.shape[0] * scale_percent / 100)
    dim = (width, height)
    # Display images
    im_src = cv2.resize(im_src, dim, interpolation = cv2.INTER_AREA)
    im_dst = cv2.resize(im_dst, dim, interpolation = cv2.INTER_AREA)
    im_out = cv2.resize(im_out, dim, interpolation = cv2.INTER_AREA)
    cv2.imshow("Source Image", im_src)
    cv2.imshow("Destination Image", im_dst)
    cv2.imshow("Warped Source Image", im_out)

    cv2.waitKey(0)