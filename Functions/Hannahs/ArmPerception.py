#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

class ColorTracker():

    def __init__(self):
        self.img_size = (640, 480)
        self.get_roi = False

        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
            }
        # World x, world y, angle of rotation
        self.cube_locs = {
            'red': [0, 0, 0],
            'blue': [0, 0, 0],
            'green': [0, 0, 0]
        }
        # self.rects = 

    def get_detected_blocks(self):
        detected_colors = []
        for color in self.cube_locs.keys():
            if self.cube_locs[color] != [0, 0, 0]:
                detected_colors.append(color)
        return detected_colors

    def get_cube_locs(self):
        return self.cube_locs


    def detect_cubes(self, img):
        ''' Main flight code. Detects red objects and draws a bounding box.'''
        desired_colors = ['red', 'blue', 'green']
        self.prepare_image(img)
        for color in desired_colors:
            self.update_cube_location(color)
        # print(self.get_detected_blocks())
        # print("detect cubes code finished. Cubes seen: {}s".format(self.cube_locs))
        return self.img_copy


    def update_cube_location(self, color):
        mask, max_contour, max_area = self.detect_color_contours(color)
        if max_area > 2500:
            self.get_bounding_box(max_contour, color)
        else:
            self.cube_locs[color] = [0, 0, 0]


    def prepare_image(self, img):
        ''' 
        Input: image from camera
        Process: preps image for color detection, saves to self.prepped_img 
        '''
        self.og_img = img 
        self.img_copy = img.copy() # make a copy of the image
        self.img_h, self.img_w = img.shape[:2]

        # Draw the center line
        cv2.line(self.img_copy, (0, int(self.img_h / 2)), (self.img_w, int(self.img_h / 2)), (0, 0, 200), 1)
        cv2.line(self.img_copy, (int(self.img_w / 2), 0), (int(self.img_w / 2), self.img_h), (0, 0, 200), 1)
        
        # Resize the image
        frame_resize = cv2.resize(self.img_copy, self.img_size, interpolation=cv2.INTER_NEAREST)
        # Blur the image
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        # Convert to LAB color space
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
        self.prepped_img = frame_lab

    def detect_color_contours(self, detect_color):
        '''
        Input: desired color string
        Performs some image processing to get a mask detecting the given color; finds contours on that image.
        Returns: masked image (1=object, 0=not object), biggest contour of that color, area of biggest contour
        '''
        frame_mask = cv2.inRange(self.prepped_img, color_range[detect_color][0], color_range[detect_color][1])
        # Erosion then dilation of mask
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8)) 
        # dilation then erosion of mask
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8)) 
        # look at the mask for contours, select the biggest
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2] 
        areaMaxContour, area_max = self.getAreaMaxContour(contours)  # 找出最大轮廓 find the largest contour
        # print("max area is: {}".format(area_max))
        return closed, areaMaxContour, area_max

    def getAreaMaxContour(self, contours):
        '''
        Directly copied from ColorTracking.py. 
        Input: list of contours
        output: the area and contour object of the largest contour
        '''
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # iterate over all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  # calculate the contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # The contour with the largest area is valid only if the area is greater than 300 to filter out the noise
                    area_max_contour = c

        return area_max_contour, contour_area_max

    def get_bounding_box(self, contour, color):
        '''
        Inputs: a contour and the color object it represents
        Draws the bounding box on the original camera image
        '''
        rect = cv2.minAreaRect(contour)
        box = np.int0(cv2.boxPoints(rect))

        roi = getROI(box)
        self.get_roi = True

        img_centerx, img_centery = getCenter(rect, roi, self.img_size, square_length)  # 获取木块中心坐标 get the coordinates of the center of the block
        world_x, world_y = convertCoordinate(img_centerx, img_centery, self.img_size) #转换为现实世界坐标 convert to real world coordinates
        self.cube_locs[color] = [world_x, world_y, rect[2]] # rect[2] is angle of rotation

        # draw box and middle point
        cv2.drawContours(self.img_copy, [box], -1, self.range_rgb[color], 2)
        cv2.putText(self.img_copy, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[color], 1) #绘制中心点 draw center point



if __name__ == '__main__':
    p = ColorTracker()
    my_camera = Camera.Camera() 
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = p.detect_cubes(img)
            cv2.imshow('Frame', frame)
            key = cv2.waitKey(1)
            if key == 27:
                break 
    my_camera.camera_close()
    cv2.destroyAllWindows()