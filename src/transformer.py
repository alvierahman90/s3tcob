import numpy as np
import cv2
from multiprocessing import Queue, Process, Pipe
from multiprocessing.connection import Connection

class Transformer:

    def __init__(self, M, p_tracker, p_parent):
        self.M = M
        self.p_tracker = p_tracker
        self.p_parent = p_parent


    def transform_point(self, point):
        """
        returns physical position on plane that a pixel at `point` is looking at

        point: tuple or list like [x, y]

        returns list like [x, y] in millimetres unless object has not processed a calibration
        image yet
        """
        return cv2.perspectiveTransform(np.array([[points]]), self.M)[0][0]

    def loop(self):
        """
        Starts loop of transforming all points received from tracker pipe
        """

        while True:
            results = self.p_tracker.recv()
            for result in results:
                for box in results.boxes():
                    pass
                    # send boxes and colour
