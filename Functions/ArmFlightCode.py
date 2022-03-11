#!/usr/bin/python3
# coding=utf8

# Libs
import sys
sys.path.append('/home/pi/ArmPi/')
sys.path.append('/home/pi/ArmPi/Functions/aruco_marker/')
import time
import logging

# Locals
from aruco_marker_detector import ArucoMarkerDetector
from ArmPerception import ColorTracker
from ArmActions import ArmMover
import Camera
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)



class Flight():
    def __init__(self):
        self.aruco = ArucoMarkerDetector(0)
        self.movement = ArmMover()

        self.fly()

    def fly(self):
        self.movement.go_to_initial_position()
        self._wait_for_aruco()
        # Grab a block. Find aruco location.
        self.movement.grasp_obj_at_pose(self.movement.poses['blue'], 'cube', 'ground', lift=True)
        aruco_loc = self.aruco.detectMarkerXYPosition()
        # Place block on truck.
        self.movement.drop_obj_in_loc(aruco_loc, 'cube', obj_height='truck')
        self.movement.go_to_initial_position()
        self.movement.wall_move()

    def _wait_for_aruco(self):
        """Continually checks for marker. Returns when one is seen."""
        aruco_loc = None
        while aruco_loc is None:
            aruco_loc = self.aruco.detectMarkerXYPosition()


    #     self.blocks_detected = False
    #     self.blocks_present = True
    #     couter = 0
    #     self.movement.go_to_initial_position()
    #     while self.blocks_present == True:
    #         img = self.cam.frame
    #         if img is not None:
    #             couter +=1
    #             frame = self.perception.detect_cubes(img)
    #             cv2.imshow('Frame', frame)
    #             if couter > 1: # first frame will not detect cubes
    #                 self.sort_next_block()
    #             key = cv2.waitKey(1)
    #             if key == 27:
    #                 break
    #     self.cam.camera_close()
    #     cv2.destroyAllWindows()
    #
    # def sort_next_block(self):
    #     blocks = self.perception.get_detected_blocks()
    #     print("Blocks detected: {}".format(blocks))
    #     if not blocks:
    #         print("No blocks detected.")
    #         self.blocks_present = False
    #     else:
    #         color = blocks[0]
    #         print("Sorting {} block next.".format(color))
    #         block_loc = self.perception.cube_locs[color]
    #         if self.movement.check_if_reachable(block_loc, 'cube'):
    #             self.movement.grasp_obj_at_coords(block_loc, 'cube')
    #             self.movement.drop_cube_in_square(color)
    #             self.movement.go_to_initial_position()

if __name__ == "__main__":
    f = Flight()
    
    