# -*- coding: utf-8 -*-
from __future__ import (
    division,
    print_function,
)

import skimage.data
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import selectivesearch
import cv2
import sys
import os
from scipy import misc
import numpy as np

#os.chdir(os.path.dirname(sys.argv[0]))
#path = os.getcwd()+'/'

def get_scale(x): return x

def get_sigma(x): return x

def get_min_size(x): return x

def main():
    cv2.namedWindow('trackbars')
    # create trackbars for color change
    cv2.createTrackbar('scale','trackbars',2000,2000,get_scale)
    cv2.createTrackbar('0.1*sigma','trackbars',11,30,get_sigma)
    cv2.createTrackbar('min_size','trackbars',500,500,get_min_size)
	
    cap = cv2.VideoCapture(0)
    while(1):
        ret, frame = cap.read()
        img = cv2.resize(frame, (320, 240), interpolation = cv2.INTER_LINEAR)
        height, width, channels = img.shape

        #scale = ((height > width) and height or width) * 3
        #sigma = 1
        #min_size = int(scale * 0.1)
    
        scale = cv2.getTrackbarPos('scale','trackbars')#1000
        sigma = 0.1 * cv2.getTrackbarPos('0.1*sigma','trackbars')#1.7
        min_size = cv2.getTrackbarPos('min_size','trackbars')#200
        
        #cv2.waitKey(100000)
		
        #img = misc.imread('4.jpg')
        # perform selective search
        img_lbl, regions = selectivesearch.selective_search(img , scale=scale, sigma=sigma, min_size=min_size)
        #img, scale=300, sigma=0.9, min_size=10)
        candidates = set()
        for r in regions:
            # excluding same rectangle (with different segments)
            if r['rect'] in candidates:
                continue
            # excluding regions smaller than 2000 pixels
            if r['size'] < 2000:
                continue
            # distorted rects
            x, y, w, h = r['rect']
            if w / h > 1.2 or h / w > 1.2:
                continue
            candidates.add(r['rect'])

        #r,g,b = cv2.split(img)
        #img = cv2.merge((b,g,r))
	
        for x, y, w, h in candidates:
            print(x, y, w, h)
            cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
	
        cv2.imshow('image',img)
        key = cv2.waitKey(0) & 0xFF
        if key == ord('q'):
            break
	
    #cv2.waitKey(100000)
	
if __name__ == "__main__":
    main()