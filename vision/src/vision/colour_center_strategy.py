#!/usr/bin/env python3
from vision.center_strategy_base import CenterStrategyBase
import cv2
import numpy as np

class ColourCenterStrategyClass(CenterStrategyBase):
    def __init__(self, lower=np.array([60, 35, 140]), upper=np.array([180, 255, 255])):
        super().__init__()
        self.lower_threshold = lower
        self.upper_threshold = upper


    def setLowerThreshhold(self, lower):
        self.lower_threshold = lower


    def setUpperThreshold(self, upper):
        self.upper_threshold = upper

        
    def findCenter(self, image):
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        img_mask = cv2.inRange(img_hsv, self.lower_threshold, self.upper_threshold)

        img_mask = cv2.erode(img_mask, None, iterations=2)
        img_mask = cv2.dilate(img_mask, None, iterations=2)

        img_masked = cv2.bitwise_and(img_hsv, img_hsv, mask=img_mask)

        img_gray = cv2.cvtColor(img_masked, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(img_gray, 1, 255, cv2.THRESH_BINARY)
        M = cv2.moments(thresh)
        mass_center = (mu['m10'] / (mu['m00'] + 1e-5), mu['m01'] / (mu['m00'] + 1e-5))

        return mass_center
