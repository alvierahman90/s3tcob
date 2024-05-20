#!/usr/bin/env python3

print("importing libraries...")
import cv2
from multiprocessing import Queue, Process, Pipe
from multiprocessing.connection import Connection
import sys
import argparse
import numpy as np
import os

from calibrater import Calibrater, PaddingMillimetres

print("imported libraries")


def args():
    parser = argparse.ArgumentParser()
    parser.add_argument("video_path")
    parser.add_argument("-m", "--model_path", default="yolov8n.pt")
    parser.add_argument(
        "-p",
        "--padding",
        default=None,
        help="Padding (in millimetres) as 4 comma separated integers in the top left,top,right,bottom. Overrides --padding.[left, top, right, bottom] options.",
    )
    parser.add_argument(
        "-l",
        "--padding.left",
        type=int,
        default=210,
        dest="padding_left",
        help="Left padding (millimetres)",
    )
    parser.add_argument(
        "-t",
        "--padding.top",
        type=int,
        default=200,
        dest="padding_top",
        help="Top padding (millimetres)",
    )
    parser.add_argument(
        "-r",
        "--padding.right",
        type=int,
        default=0,
        dest="padding_right",
        help="Right padding (millimetres)",
    )
    parser.add_argument(
        "-b",
        "--padding.bottom",
        type=int,
        default=100,
        dest="padding_bottom",
        help="Bottom padding (millimetres)",
    )
    parser.add_argument(
        "--px-per-mm",
        type=int,
        default=2,
        dest="px_per_mm",
        help="Pixels in the output image per real life millimetre",
    )
    parser.add_argument(
        "--board.square-mm",
        type=float,
        default=20,
        dest="board__square_mm",
        help="Size of squares of the calibration board in millimetres",
    )
    parser.add_argument(
        "--board.dimensions.x",
        type=int,
        default=3,
        dest="board__dimensions__x",
        help="Width of calibration board (number of crosses)",
    )
    parser.add_argument(
        "--board.dimensions.y",
        type=int,
        default=9,
        dest="board__dimensions__y",
        help="Height of calibration board (number of crosses)",
    )

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

    if args.padding is not None:
        padding = args.padding.split(",")
        padding_left = int(padding[0])
        padding_top = int(padding[1])
        padding_right = int(padding[2])
        padding_bottom = int(padding[3])
    else:
        padding_left = args.padding_left
        padding_top = args.padding_top
        padding_right = args.padding_right
        padding_bottom = args.padding_bottom

    calibrater = Calibrater(
        px_per_mm=args.px_per_mm,
        square_mm=args.board__square__mm,
        board_dims=[args.board__dimensions__x, args.board__dimensions__y],
        padding=PaddingMillimetres(
            padding_left, padding_top, padding_right, padding_bottom
        ),
    )

    print("Calibrating...")
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
