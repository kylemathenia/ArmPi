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


class Movement:
    def __init__(self):
        self.track = False
        self.get_roi = False
        self.unreachable = None
        self.isRunning = True
        self.detected_color = 'None'
        self.action_finished = True
        self.start_pick_up = False
        self.first_move = True
        self.grip_close_ang = 500  # The angle at which the gripper closes when gripping
        self.buzzer_delay = 0.1
        self.coordinate = {
            'red': (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5, 1.5),
            'blue': (-15 + 0.5, 0 - 0.5, 1.5),
        }
        self.rotation_angle = 0
        self.world_x_ave = 0
        self.world_y_ave = 0
        self.world_x = 0
        self.world_y = 0

        self.AK = ArmIK()
        self.initMove()

    def move(self,perception_data):
        self.update_values(perception_data)
        if self.isRunning:
            if self.first_move and self.start_pick_up:  # When an object is first detected
                self.object_first_detected()
            elif not self.first_move and not self.unreachable:  # Not the first time an object has been detected
                self.object_already_detected()
        else:
            self.stop()
        movement_data = self.gather_data_to_send()
        return movement_data

    def update_values(self,perception_data):
        self.track = perception_data['track']
        self.isRunning = perception_data['isRunning']
        self.detected_color = perception_data['detected_color']
        self.start_pick_up = perception_data['start_pick_up']
        self.rotation_angle = perception_data['rotation_angle']
        self.world_x_ave = perception_data['world_x_ave']
        self.world_y_ave = perception_data['world_y_ave']
        self.world_x = perception_data['world_x']
        self.world_y = perception_data['world_y']

    def gather_data_to_send(self):
        data = {}
        data['action_finished'] = self.action_finished
        data['get_roi'] = self.get_roi
        data['isRunning'] = self.isRunning
        data['detected_color'] = self.detected_color
        data['start_pick_up'] = self.start_pick_up
        return data

    def object_first_detected(self):
        self.action_finished = False
        self.set_rgb(self.detected_color)
        self.setBuzzer()
        # Do not fill in the running time parameter, adaptive running time
        result = self.AK.setPitchRangeMoving((self.world_x_ave, self.world_y_ave - 2, 5), -90, -90, 0)
        if result == False:
            self.unreachable = True
        else:
            self.unreachable = False
        time.sleep(result[2] / 1000)  # The third item of the returned parameter is the time
        self.start_pick_up = False
        self.first_move = False
        self.action_finished = True

    def object_already_detected(self):
        """The main action sequence. Currently there are a bunch of sleep statements. While the sequence is running,
        no new information is coming in, which is different than the original code that used globals. This could
        be fixed by having a sequence timer direct the code flow to the current portion of the action sequence."""
        self.set_rgb(self.detected_color)
        if self.track:  # If it is the tracking phase
            self.AK.setPitchRangeMoving((self.world_x, self.world_y - 2, 5), -90, -90, 0, 20)
            time.sleep(0.02)
            self.track = False
        if self.start_pick_up:  # If the object has not moved for a while, start gripping
            self.action_finished = False
            self.pickup_action_1()
            time.sleep(0.8)
            self.pickup_action_2()
            time.sleep(2)
            self.pickup_action_3()
            time.sleep(1)
            self.pickup_action_4()
            time.sleep(1)
            result = self.pickup_action_5()
            time.sleep(result[2] / 1000)
            self.pickup_action_6()
            time.sleep(0.5)
            self.pickup_action_7()
            time.sleep(0.5)
            self.pickup_action_8()
            time.sleep(0.8)
            self.pickup_action_9()
            time.sleep(0.8)
            self.pickup_action_10()
            time.sleep(0.8)
            self.action_finished_reset()
        else:
            time.sleep(0.01)
    
    def stop(self):
        Board.setBusServoPulse(1, self.grip_close_ang - 70, 300)
        time.sleep(0.5)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        time.sleep(1.5)

    def initMove(self):
        """Move to the initial position."""
        Board.setBusServoPulse(1, self.grip_close_ang - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def set_rgb(color):
        """Set the color of the RGB lights of the expansion board to match the color to be tracked"""
        if color == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
            Board.RGB.show()
        elif color == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
            Board.RGB.show()
        elif color == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
            Board.RGB.show()
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
            Board.RGB.show()

    def setBuzzer(self):
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(self.buzzer_delay)
        Board.setBuzzer(0)



    #################################################################
    ########## Actions performed during a pickup procedure ##########
    #################################################################

    def action_finished_reset(self):
        # Finished. Return to original position.
        self.initMove()
        time.sleep(1.5)
        self.detected_color = 'None'
        self.first_move = True
        self.get_roi = False
        self.action_finished = True
        self.start_pick_up = False
        self.set_rgb(self.detected_color)

    def pickup_action_1(self):
        Board.setBusServoPulse(1, self.grip_close_ang - 280, 500)  # paws open
        # Calculate the angle by which the gripper needs to be rotated
        servo2_angle = getAngle(self.world_x_ave, self.world_y_ave, self.rotation_angle)
        Board.setBusServoPulse(2, servo2_angle, 500)

    def pickup_action_2(self):
        self.AK.setPitchRangeMoving((self.world_x_ave, self.world_y_ave, 2), -90, -90, 0, 1000)  # lower the altitude

    def pickup_action_3(self):
        Board.setBusServoPulse(1, self.grip_close_ang, 500)  # Gripper closed

    def pickup_action_4(self):
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((self.world_x_ave, self.world_y_ave, 12), -90, -90, 0, 1000)  # The robotic arm is raised

    def pickup_action_5(self):
        # Sort and place blocks of different colors
        result = self.AK.setPitchRangeMoving((self.coordinate[self.detected_color][0],
                                         self.coordinate[self.detected_color][1], 12), -90, -90, 0)
        return result

    def pickup_action_6(self):
        servo2_angle = getAngle(self.coordinate[self.detected_color][0], self.coordinate[self.detected_color][1], -90)
        Board.setBusServoPulse(2, servo2_angle, 500)

    def pickup_action_7(self):
        self.AK.setPitchRangeMoving((self.coordinate[self.detected_color][0], self.coordinate[self.detected_color][1],
                                self.coordinate[self.detected_color][2] + 3), -90, -90, 0, 500)

    def pickup_action_8(self):
        self.AK.setPitchRangeMoving((self.coordinate[self.detected_color]), -90, -90, 0, 1000)

    def pickup_action_9(self):
        Board.setBusServoPulse(1, self.grip_close_ang - 200, 500)  # Claws open to drop objects

    def pickup_action_10(self):
        self.AK.setPitchRangeMoving((self.coordinate[self.detected_color][0], self.coordinate[self.detected_color][1],
                                12), -90, -90, 0, 800)
