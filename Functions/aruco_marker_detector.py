from typing import Tuple
import atexit
import logging

import cv2
import numpy as np

from bufferless_video_capture import BufferlessVideoCapture

class ArucoMarkerDetector():
    """Class for simple aruco marker detection"""
    def __init__(self, camera_stream: int, visualize_results: bool = True) -> None:
        # Initialize dictionary of aruco markers for easy reference
        self.marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        # Start the video capture
        self.cap = BufferlessVideoCapture(camera_stream)
        # Setup camera matrix and distortion coefficients for pose estimation
        self.camera_mtx = np.array(
            [[1.18387476e+03, 0.00000000e+00, 3.48472753e+02],
            [0.00000000e+00, 1.18582193e+03, 2.12580655e+02],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
        )
        self.distortion_coefficients = np.array(
            [[-4.71751133e-01,  1.83250256e+00, -3.32571108e-04, -5.83076160e-04,  -1.55658703e+01]]
        )
        # Length of side of marker in cm
        self.marker_length = 2.628 # about 1 inch
        # For visualizing detected marker
        self.visualize_results = visualize_results
        # Cleanup visualization
        atexit.register(self.cleanup)

    def detectCornersInFrame(self, frame: np.ndarray)->Tuple:
        """Detect the relevant aruco marker corners in the given frame."""
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(frame, self.marker_dict)
        return corners, ids, rejected_img_points

    def detectMarkerFull(self)->Tuple:
        """Detect the marker from the camera.
        Return the marker pose relative to the camera and corners in the image."""
        # Read in the latest frame
        frame = self.cap.read()
        # Look for the aruco marker
        corners, ids, rejected_img_points = self.detectCornersInFrame(frame)
        # Estimate the pose if marker was found
        rvecs, tvecs, _objPoints = [], [], []
        marker_detected = False
        if len(corners) > 0:
            rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_mtx, self.distortion_coefficients)
            marker_detected = True
        else:
            logging.info("No marker detected.")

        if self.visualize_results:
            result_frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            if marker_detected:
                result_frame = cv2.aruco.drawAxis(result_frame, self.camera_mtx, self.distortion_coefficients, rvecs[0], tvecs[0], self.marker_length)
            cv2.imshow("Aruco Marker Detector", result_frame)
            cv2.waitKey(1)

        return corners, ids, rejected_img_points, rvecs, tvecs, _objPoints

    def detectMarkerPose(self)->Tuple[np.ndarray, np.ndarray]:
        """Detect the marker pose relative to the camera.
        Return a translation and rotation vector for the aruo marker."""
        _, _, _, rvecs, tvecs, _ = self.detectMarkerFull()
        if len(rvecs) > 0 and len(tvecs) > 0:
            return rvecs[0][0], tvecs[0][0]
        else:
            logging.warning("No marker detected. Returning (None, None)")
            return None, None

    def detectMarkerXYPosition(self)->np.ndarray:
        """Detect the marker's xy position relative to the robot base
        Return an [x,y] vector for the robot's position if the marker
        is detected. Otherwise, return None"""
        _, tvec = self.detectMarkerPose()
        if tvec is not None:
            return np.array([tvec[0]+1, -tvec[1] + 20])
        else:
            return None

    def cleanup(self)->None:
        cv2.destroyAllWindows()
        return None

if __name__ == "__main__":
    # Setup logging format
    logging_format = "%(asctime)s [%(levelname)s] %(funcName)s(): %(message)s"
    logging.basicConfig(format=logging_format, level=logging.INFO,
        datefmt="%H:%M:%S")

    # Run detector
    detector = ArucoMarkerDetector(camera_stream=0, visualize_results=True)
    while True:
        xypos = detector.detectMarkerXYPosition()
        print("Position: {}".format(xypos))