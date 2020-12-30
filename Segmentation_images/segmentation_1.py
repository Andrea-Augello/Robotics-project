#!/usr/bin/env python
import cv2
import sys

im = cv2.imread(sys.argv[1])
im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

otsu_threshold, thresh = cv2.threshold( im, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_L1)
#cv2.drawContours(im, contours, -1, (0,255,0), 3)
clusters=[]
for c in contours:
    M = cv2.moments(c)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    clusters.append(
            {'center':(cx,cy),
                'area':M['m00'],
                'contour':c})
    cv2.circle(im, (cx,cy), 2, (0,255,0), 1)

#cv2.imshow('f',im)
#cv2.waitKey(0)
return clusters
