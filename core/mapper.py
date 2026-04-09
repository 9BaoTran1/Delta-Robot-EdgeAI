import cv2
import numpy as np
import os

class RobotMapper:
    def __init__(self, matrix_path="models/calibration_matrix.npy"):
        self.matrix = None
        if os.path.exists(matrix_path):
            try:
                self.matrix = np.load(matrix_path)
                print(f"Loaded calibration matrix from {matrix_path}")
            except Exception as e:
                print(f"Error loading matrix: {e}")

    def pixel_to_robot(self, u, v):
        if self.matrix is None:
            return None, None
        point = np.array([[[u, v]]], dtype="float32")
        robot_point = cv2.perspectiveTransform(point, self.matrix)
        return robot_point[0][0][0], robot_point[0][0][1]
