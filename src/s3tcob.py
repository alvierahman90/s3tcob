#!/usr/bin/env python3

import cv2
from multiprocessing import Queue, Process, Pipe
from multiprocessing.connection import Connection
import sys
from ultralytics import YOLO
import argparse

from .tracker import Tracker
from .calibrater import Calibrater


def args():
    parser = argparse.ArgumentParser()
    parser.add_argument("video_path")
    parser.add_argument("-m", "--model_path", default="yolov8n.pt")

    return parser.parse_args()


def main(args):
    print("Initialising system and loading model...")
    capture = cv2.VideoCapture(args.video_path)
    model = YOLO(args.model_path, verbose=False)
    tracker = Tracker(model)
    calibrater = Calibrater(px_per_mm=10, square_mm20, board_dims=[6, 8])
    print("Initialised {capture=} {model=} {tracker=} {calibrater=}")

    print(f"Calibrating...")

    while capture.isOpened():
        success, frame = capture.read()
        if not success:
            continue

        M = calibrater.get_calibration_transform(frame)
        break

    print(f"Calibrated {M=}")

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
