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
import logging
from ArmPerception import ColorTracker
from ArmActions import ArmMover

#TODO
# import car script
import concurrent.futures

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


def run():
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        thread1 = executor.submit(car_script.main())
        thread2 = executor.submit(arm_flight.main())
    thread1.result()
    thread2.result()



class Flight():

    def __init__(self):
        self.p = ColorTracker()
        self.m = ArmMover()

        self.blocks_detected = False

        self.cam = Camera.Camera() 
        self.cam.camera_open()
        self.blocks_present = True
        ctr = 0
        while self.blocks_present == True:
            img = self.cam.frame
            if img is not None:
                ctr +=1 
                frame = self.p.detect_cubes(img)
                cv2.imshow('Frame', frame)
                if ctr > 1:
                    self.sort_next_block()
                key = cv2.waitKey(1)
                if key == 27:
                    break 
        self.cam.camera_close()
        cv2.destroyAllWindows()

    def sort_next_block(self):
        blocks = self.p.get_detected_blocks()
        print("Blocks detected: {}".format(blocks))
        if not blocks:
            print("No blocks detected.")
            self.blocks_present = False 
        else:
            color = blocks[0]
            print("Sorting {} block next.".format(color))
            block_loc = self.p.cube_locs[color]
            if self.m.check_if_reachable(block_loc):
                self.m.grasp_cube_at_coords(block_loc)
                self.m.drop_cube_in_square(color)




if __name__ == "__main__":
    f = Flight()
    
    