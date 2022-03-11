#!/usr/bin/python3
# coding=utf8

# Libs
import sys
sys.path.append('/home/pi/ArmPi/')

# Locals
from aruco_marker_detector import ArucoMarkerDetector
from ArmActions import ArmMover
from ArmIK.ArmMoveIK import *

if sys.version_info.major == 2:
    print('Please run this program with sudo python3!')
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

if __name__ == "__main__":
    f = Flight()
    
    