#!/usr/bin/python3
# coding=utf8
import sys

from numpy import square

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
import logging
from ArmPerception import ColorTracker

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


class ArmMover():

    def __init__(self):
        self.AK = ArmIK()
        self.gripper_vals = {
            'cube': 500,
            'open': 220,
            'wall': 600
        }

        self.heights = {
            'cube': {
                'ground': 1.5,
                'truck': 8.5
            },
            'wall': {
                'ground': 7,
                'drag': 8
            }
        }

        self.range_rgb = {
            'blue': (0, 0, 255),
            'red': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }

        # Coordinates of colored boxes on mat
        self.poses = {
            'red': [[-15 + 0.5, 12 - 0.5], 0],
            'green': [[-15 + 0.5, 6 - 0.5], 0],
            'blue': [[-15 + 0.5, 0 - 0.5], 0],
            'wall_block': [[15, 15], 0],
            'wall_away': [[15, 0], 0]
        }

    def go_to_initial_position(self):
        ''' Send the robot to the "home" position.'''
        logging.info("Moving arm to initial position.")
        self.open_gripper()
        time.sleep(1.0)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        time.sleep(1.5)

    def set_lights_to_color(self, color=None):
        # print(self.range_rgb[color])
        if color is not None:
            Board.RGB.setPixelColor(0, Board.PixelColor(*self.range_rgb[color]))
            Board.RGB.setPixelColor(1, Board.PixelColor(*self.range_rgb[color]))
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))

        Board.RGB.show()

    def check_if_reachable(self, pose, object):
        # print("Checking if pose {} is reachable".format(pose))
        # result = self.AK.setPitchRangeMoving((pose[0], pose[1], 7), -90, -90, 0)
        result = self.AK.setPitchRangeMoving((*pose[0], self.heights[object]['ground']), -90, -90, 0)
        if result == False:
            # print("not reachable.")
            return False
        else:
            # print("reachable!")
            return True
        # time.sleep(result[2]/1000)

    def move_to_loc(self, loc, height, duration=1.5):
        '''Moves gripper to location provided
        loc is [x, y], height is z
        optional input: duration of movement'''
        self.AK.setPitchRangeMoving((loc[0], loc[1], height), -90, -90, 0)
        time.sleep(duration)

    def set_gripper_angle(self, angle):
        '''Sets gripper angle based on angle of object wrt world x plane'''
        Board.setBusServoPulse(2, angle, 500)
        time.sleep(0.8)

    def straighten_gripper(self, loc, des_angle=0):
        '''Straightens gripper to 0 degrees, based on the location of the end effector in world
        loc is [x, y]'''
        straight_angle = getAngle(loc[0], loc[1], des_angle)
        self.set_gripper_angle(straight_angle)

    def grasp_obj_at_pose(self, pose, object, obj_height, lift=True):
        '''Picks up the object located at the coordinates given (pose: [[x, y], angle])'''
        # move to above the object and open the gripper
        loc = pose[0]
        self.move_to_loc(loc, self.heights[object][obj_height] + 8)
        self.open_gripper()
        # Set the gripper angle
        gripper_angle = getAngle(*loc, pose[1])  # Calculate the angle by which the gripper needs to be rotated
        self.set_gripper_angle(gripper_angle)
        # Lower to around cube
        self.move_to_loc(loc, self.heights[object][obj_height])
        self.close_gripper(object)
        if lift == True:
            self.move_to_loc(loc, self.heights[object][obj_height] + 8)
        else:
            self.move_to_loc(loc, self.heights[object]['drag'])

    def wall_move(self, frompos='wall_block', topos='wall_away'):
        '''Moves to or from initial position to the "away" position.'''
        og_pose = self.poses[frompos]
        new_pose = self.poses[topos]
        self.grasp_obj_at_pose(og_pose, 'wall', lift=True)
        # self.straighten_gripper(og_pose[0], og_pose[1])
        self.move_to_loc(new_pose[0], self.heights['wall']['drag'])
        self.move_to_loc(new_pose[0], self.heights['wall']['ground'], duration=0.5)
        self.straighten_gripper(*new_pose[0])
        self.open_gripper()
        self.move_to_loc(*new_pose[0], 12)

    def drop_cube_in_square(self, square_color):
        loc = self.coordinate[square_color][0]
        self.drop_obj_in_loc(loc, 'cube')

    def drop_obj_in_loc(self, loc, object, obj_height='ground'):
        '''Places an (already grasped) object in a location'''
        # move to location over object
        self.move_to_loc(loc, self.heights[object][obj_height] + 8)
        # set the angle to be straight
        straight_angle = getAngle(*loc, -90)
        self.set_gripper_angle(straight_angle)

        # drop object
        self.move_to_loc(loc, self.heights[object][obj_height] + 1, duration=0.5)
        self.open_gripper()
        self.move_to_loc(loc, self.heights[object][obj_height] + 8, duration=0.8)


    def open_gripper(self):
        Board.setBusServoPulse(1, self.gripper_vals['open'], 500)
        time.sleep(0.8)

    def close_gripper(self, object):
        '''closes the gripper on object. Use 'cube' for cube or 'wall' for wall.'''
        Board.setBusServoPulse(1, self.gripper_vals[object], 500)
        time.sleep(0.8)


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.INFO)
    p = ColorTracker()
    m = ArmMover()
    m.go_to_initial_position()
    time.sleep(0.5)

    # detected_blocks = p.get_detected_blocks()