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

class Perception:
    def __init__(self,perception_mode='tracking',target_color='red'):
        self.set_perception_mode_func(perception_mode)
        self.target_color = (target_color,)
        self.isRunning = True
        self.size = (640, 480)
        self.img = None
        self.img_copy = None
        self.frame_gb = None
        self.get_roi = False
        self.start_pick_up = False
        self.roi = ()
        self.rect = None
        self.count = 0
        self.track = False
        self.detected_color = 'None'
        self.action_finished = True
        self.start_count_t1 = True
        self.center_list = []
        self.rotation_angle = 0
        self.world_x_ave = 0
        self.world_y_ave = 0
        self.world_x = 0
        self.world_y = 0
        self.last_x = 0
        self.last_y = 0
        self.t1 = 0
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }

        self.camera = Camera.Camera()
        self.camera.camera_open()


    ############### Main functions ###############
    def perceive(self,movement_data):
        """The main function. Updates values and runs whatever perception function is assigned."""
        self.update_values(movement_data)
        self.img = self.camera.frame
        if self.img is not None:
            self.perception_func()
            cv2.imshow('Frame', self.img)
            self.check_if_break()
        perception_data = self.gather_data_to_send()
        return perception_data

    def update_values(self,movement_data):
        self.action_finished = movement_data['action_finished']
        self.get_roi = movement_data['get_roi']
        self.isRunning = movement_data['isRunning']
        self.detected_color = movement_data['detected_color']
        self.start_pick_up = movement_data['start_pick_up']

    def gather_data_to_send(self):
        data = {}
        data['track'] = self.track
        data['isRunning'] = self.isRunning
        data['detected_color'] = self.detected_color
        data['start_pick_up'] = self.start_pick_up
        data['rotation_angle'] = self.rotation_angle
        data['world_x_ave'] = self.world_x_ave
        data['world_y_ave'] = self.world_y_ave
        data['world_x'] = self.world_x
        data['world_y'] = self.world_y
        return data


    ############### Perception functions ###############

    def set_perception_mode_func(self,perception_mode):
        if perception_mode == 'tracking':
            self.perception_func = self.tracking_mode_func
        elif perception_mode == 'sorting':
            self.perception_func = self.sorting_mode_func
        elif perception_mode == 'palletizing':
            self.perception_func = self.palletizing_mode_func
        else:
            raise KeyError

    def tracking_mode_func(self):
        self.preprocess_img()
        self.check_if_area_and_start_pick_up()
        areaMaxContour, area_max = self.find_largest_area()

        if area_max > 2500:  # have found the largest area
            distance = self.update_coords(areaMaxContour)
            self.track = True
        if area_max > 2500 and self.action_finished:
            self.check_average_points(distance)

    def sorting_mode_func(self):
        pass

    def palletizing_mode_func(self):
        pass

    ############### Supporting functions ###############

    def preprocess_img(self):
        self.img_copy = self.img.copy()
        img_h, img_w = self.img.shape[:2]
        cv2.line(self.img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(self.img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        frame_resize = cv2.resize(self.img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        self.frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)

    def check_if_area_and_start_pick_up(self):
        """If an area is detected with a recognized object, the area is detected until there are none"""
        if self.get_roi and self.start_pick_up:
            self.get_roi = False
            self.frame_gb = getMaskROI(self.frame_gb, self.roi, self.size)

    def check_if_area_and_not_start_pick_up(self):
        """If an area is detected with a recognized object, the area is detected until there are none"""
        if self.get_roi and not self.start_pick_up:
            self.get_roi = False
            self.frame_gb = getMaskROI(self.frame_gb, self.roi, self.size)

    def find_largest_area(self):
        frame_lab = cv2.cvtColor(self.frame_gb, cv2.COLOR_BGR2LAB)  # Convert image to LAB space
        area_max = 0
        areaMaxContour = 0
        if not self.start_pick_up:
            self.detected_color = self.target_color[0]
            frame_mask = cv2.inRange(frame_lab,
                                     self.range_rgb[self.detected_color][0], self.range_rgb[self.detected_color][
                                         1])  # Bitwise operations on the original image and mask
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # open operation
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # closed operation
            contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[
                -2]  # find the outline
            areaMaxContour, area_max = self.getAreaMaxContour(contours)  # find the largest contour
            return areaMaxContour, area_max


    def update_coords(self,areaMaxContour):
        self.rect = cv2.minAreaRect(areaMaxContour)
        box = np.int0(cv2.boxPoints(self.rect))
        roi = getROI(box)  # get roi region
        get_roi = True
        # Get the coordinates of the center of the block
        img_centerx, img_centery = getCenter(self.rect, roi, self.size, square_length)
        # Convert to real world coordinates
        self.world_x, self.world_y = convertCoordinate(img_centerx, img_centery, self.size)
        cv2.drawContours(self.img, [box], -1, self.range_rgb[self.detected_color], 2)
        cv2.putText(self.img, '(' + str(self.world_x) + ',' + str(self.world_y) + ')', (
            min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[self.detected_color], 1)  # draw center point
        # Compare the last coordinates to determine whether to move
        distance = math.sqrt(pow(self.world_x - self.last_x, 2) + pow(self.world_y - self.last_y, 2))
        self.last_x, self.last_y = self.world_x, self.world_y
        return distance

    def check_average_points(self,distance):
        if distance < 0.3:
            self.center_list.extend((self.world_x, self.world_y))
            self.count += 1
            if self.start_count_t1:
                self.start_count_t1 = False
                self.t1 = time.time()
            if time.time() - self.t1 > 1.5:
                self.rotation_angle = self.rect[2]
                self.start_count_t1 = True
                self.world_x_ave, self.world_y_ave = np.mean(np.array(self.center_list).reshape(self.count, 2), axis=0)
                self.count = 0
                self.center_list = []
                self.start_pick_up = True
        else:
            self.t1 = time.time()
            self.start_count_t1 = True
            self.count = 0
            self.center_list = []

    def check_if_break(self):
        key = cv2.waitKey(1)
        if key == 27:
            self.isRunning = False

    def getAreaMaxContour(self,contours):
        """Find the contour with the largest area. Argument is a list of contrours."""
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # iterate over all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate the contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # The contour with the largest area is valid only if the area is greater than 300 to filter out the noise
                    area_max_contour = c

        return area_max_contour, contour_area_max  # returns the largest contour


    def shutdown(self):
        self.camera.camera_close()
        cv2.destroyAllWindows()