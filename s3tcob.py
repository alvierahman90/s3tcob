#!/usr/bin/env python3

import numpy as np
import cv2
from multiprocessing import Queue, Process, Pipe
from multiprocessing.connection import Connection
import sys
from ultralytics import YOLO
import argparse


class Tracker:

    def __init__(self, model: YOLO, device=0):
        self.model = model
        self.device = device
        self.queue = Queue(20)

    def push_frame(self, frame):
        """
        queue a new frame to be processed

        frame: image to add to queue
        """
        if self.queue.full():
            self.queue.get()
        self.queue.put(frame)

    def _process_frame(self):
        """
        process next frame in queue (blocks if no frame in queue)
        """
        return self.model.track(self.queue.get(),
                                persist=True,
                                device=self.device)

    def loop(self, pipe: Connection):
        """
        Start a tracking loop.

        pipe: child pipe to receive frames from and send results to
        """
        while True:
            image = pipe.recv()
            results = self.new_frame(image)
            pipe.send(results)

    def mt_loop(self):
        """
        Starts the model loop function in a separate python process

        returns (handle, pipe)
        where:
            handle: multiprocessing.Process is the handle of the process that's
                been started
            pipe: multiprocessing.connection.Connection
                a pipe to send frames to and receive segmentation output
        """

        (parent_p, child_p) = Pipe()
        handle = Process(target=self.loop, args=(child_p))
        handle.start()

        return (handle, parent_p)


class Calibrater:

    def __init__(self, px_per_mm, square_mm, board_dims):
        self.px_per_mm = px_per_mm
        self.square_mm = square_mm
        self.board_dims = board_dims

        square_px = px_per_mm * square_mm
        self.padding_px = 3 * square_px
        grid_x_px = square_px * (board_dims[0] - 1)
        grid_y_px = square_px * (board_dims[1] - 1)
        print(f"{board_dims[0]=} {board_dims[1]} {grid_x_px=} {grid_y_px=}")
        self.pts2 = np.float32([
            [self.padding_px, self.padding_px],  # top left
            [self.padding_px + grid_x_px, self.padding_px],  # top right
            [self.padding_px, self.padding_px + grid_y_px],  # bottom left
            [self.padding_px + grid_x_px,
             self.padding_px + grid_y_px]  # bottom right
        ])

    def _pts1(self, corners):
        """
        returns the 4 outer corners of the chessboard

        corners: an array of corners that are sorted according to
            https://github.com/alvierahman90/MMME4085_Colour_Identification/tree/main/calibration#cv2findchessboardcorners-output
             (opencv.findChessboardCorners does this for you)
        """
        return np.float32([
            corners[0], corners[self.board_dims[0] - 1],
            corners[-self.board_dims[0]], corners[-1]
        ])

    def get_calibration_transform(self, image):
        """
        image: opencv image

        returns tuple of (ok, M, corners) where
        - ok is a bool that is True if function completed without error
        - M is the transformation matrix (None if ok == False)
        - corners is the array of positions of corners of chessboard in
            unwarped image (None if ok == False)
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.board_dims, None)

        if not ret:
            return (False, None, None)

        # selecting the corners we care about
        pts1 = self._pts1(corners)

        M = cv2.getPerspectiveTransform(pts1, self.pts2)

        return (True, M, corners)

    def get_warped_image(self,
                         image,
                         x_extension_mm=None,
                         y_extension_mm=None):
        """
        get warped image based on calibrated transformation matrix

        image: opencv image
        x_extension_mm: millimetres to extend the image past the chessboard
        y_extension_mm: millimetres to extend the image past the chessboard
        """

        if x_extension_mm is None:
            x_extension = 2 * self.grid_x_px
        else:
            x_extension = self.px_per_mm * x_extension_mm

        if y_extension_mm is None:
            y_extension = 2 * self.grid_y_px
        else:
            y_extension = self.px_per_mm * y_extension_mm

        img_x_px = 2 * self.padding_px + 1 * self.grid_x_px + x_extension
        img_y_px = 2 * self.padding_px + 1 * self.grid_y_px + y_extension

        (ok, M, corners) = self.get_calibration_transform(image)
        if not ok:
            return (False, None)

        # plot chess board points
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_MAX_ITER, 30,
                    0.001)
        corners2 = cv2.cornerSubPix(image, corners, (11, 11), (-1, -1),
                                    criteria)
        cv2.drawChessboardCorners(image, self.board_dims, corners2)
        for pt in self._pts1(corners):
            cX, cY = pt[0][0], pt[0][1]
            center = (int(cX), int(cY))
            radius = 10
            colour = (0, 0, 255)
            image = cv2.circle(image, center, radius, colour, 3)

        image = cv2.warpPerspective(image, M, (img_x_px, img_y_px))
        return (True, image)


def args():
    parser = argparse.ArgumentParser()
    parser.add_argument("video_path")
    parser.add_argument("-m", "--model_path", default="yolov8n.pt")

    return parser.parse_args()


def main(args):
    capture = cv2.VideoCapture(args.video_path)
    model = YOLO(args.model_path, verbose=False)
    tracker = Tracker(model)
    calibrater = Calibrater(10, 20, [6, 8])

    (tracker_handle, p_tracker) = tracker.mt_loop()

    while capture.isOpened():
        success, frame = capture.read()

        if not success:
            break

        tracker.push_frame(frame)

        if p_tracker.poll():
            annotated = p_tracker.recv().plot()
            cv2.imshow("tracking", annotated)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


if __name__ == "__main__":
    sys.exit(main(args()))
