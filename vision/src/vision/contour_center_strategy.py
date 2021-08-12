#!/usr/bin/env python3
import center_strategy_base as csb
import cv2

class ContourCenterStrategyClass(csb.CenterStrategyBase):
    def __init__(self):
        super().__init__()


    def findCenter(self, image):

        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        img_gray = cv2.blur(img_gray, (3,3))

        img_thresh = cv2.adaptiveThreshold(img_gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 199, 5)
        # canny_output = cv2.adaptiveThreshold(img_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 199, 5)

        max_contour = None
        # canny_output = cv2.Canny(img_gray, thresh, thresh * 2) # canny edge detection
        contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_contour = max(contours, key = cv2.contourArea)

        if max_contour != None:
            mu = cv2.moments(max_contour)
            # add 1e-5 to avoid division by zero -> opencv tutorial method
            mass_center = (mu['m10'] / (mu['m00'] + 1e-5), mu['m01'] / (mu['m00'] + 1e-5))
            return mass_center

        return (-1, -1) # if no such contour exists
