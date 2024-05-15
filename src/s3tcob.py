#!/usr/bin/env python3

print("importing libraries...")
import cv2
from multiprocessing import Queue, Process, Pipe
from multiprocessing.connection import Connection
import sys
import argparse
import numpy as np
import os

from calibrater import Calibrater

print("imported libraries")


def args():
    parser = argparse.ArgumentParser()
    parser.add_argument("video_path")
    parser.add_argument("-m", "--model_path", default="yolov8n.pt")

    return parser.parse_args()


def tracking_loop(video_path: os.PathLike, parent_p: Connection, model="yolov8n.pt"):
    print("tracking_loop: importing yolo")
    from ultralytics import YOLO

    print("tracking_loop: imported yolo")

    print("tracking_loop: loading model")
    m = YOLO(model)
    print("tracking_loop: model loaded, starting video capture")
    capture = cv2.VideoCapture(video_path)
    print("tracking_loop: video capture created")

    while capture.isOpened():
        success, frame = capture.read()
        if not success:
            return

        results = m.track(frame, verbose=False)
        ret = []
        for result in results:
            for box in result.boxes:
                p = box.xyxy
                ret.append(
                    {
                        "class": box.cls,
                        "xyxy": list(p),
                    }
                )
        # print(f"sending results {len(ret)=}")
        parent_p.send(ret)
        cv2.imshow("boxes", results[0].plot())
        cv2.waitKey(1)


def transformer_loop(tracker_p: Connection, output_p: Connection, M: np.array):
    def perspective_transform(points):
        try:
            return cv2.perspectiveTransform(np.array([[points]]), M)[0][0]
        except:
            return np.array([0, 0])

    while True:
        results = tracker_p.recv()
        ret = []
        for result in results:
            ret.append(
                {
                    "class": result["class"],
                    "xyxy": result["xyxy"],
                    "xyxy_transformed": [
                        perspective_transform(result["xyxy"][:2]),
                        perspective_transform(result["xyxy"][2:]),
                    ],
                }
            )

        # print(f"{ret=}")
        output_p.send(ret)


def main(args):
    print("Initialising system and loading model...")
    calibrater = Calibrater(px_per_mm=10, square_mm=20, board_dims=[3, 9])

    print(f"Calibrating...")
    capture = cv2.VideoCapture(args.video_path)

    while capture.isOpened():
        success, frame = capture.read()
        if not success:
            continue

        (ok, M, corners) = calibrater.get_calibration_transform(frame)
        print("{ok=} {M=} {corners=}")
        if not ok:
            continue

        break

    print(f"Calibrated {M=}")
    capture.release()

    print("Creating tracker thread")
    q = Queue()
    (tracker_p, tracker_child_p) = Pipe()
    tracker_handle = Process(
        target=tracking_loop, args=(args.video_path, tracker_child_p)
    )
    tracker_handle.start()

    print("Creating transformer loop")
    (xf_p, xf_child_p) = Pipe()
    xf_handle = Process(target=transformer_loop, args=(tracker_p, xf_child_p, M))
    xf_handle.start()

    while True:
        if xf_p.poll():
            recv = xf_p.recv()
            print(f"xf_p.recv()={recv}")

        # if transformer_p.poll():
        #    recv = transformer_p.recv()
        #    annotated = recv["results"].plot()
        #    processed = recv["processed"]
        #    print(f"{processed=}")
        #    cv2.imshow("tracking", annotated)


if __name__ == "__main__":
    sys.exit(main(args()))
