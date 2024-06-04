#!/usr/bin/env python3

print("importing libraries...")
import cv2
from multiprocessing import Queue, Process, Pipe
from multiprocessing.connection import Connection
import sys
import argparse
import numpy as np
import os
import serial
import time

from calibrater import Calibrater, PaddingMillimetres

print("imported libraries")


def args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--video_path", default="/dev/video0")
    parser.add_argument("-m", "--model_path", default="yolov8n.pt")
    parser.add_argument(
        "-s", "--serial.port", default="/dev/ttyUSB0", dest="serial_port"
    )
    parser.add_argument(
        "--shaker-table.pwm.duty", default=100, type=int, dest="shaker_table__pwm__duty"
    )
    parser.add_argument(
        "--conveyor.pwm.duty", default=255, type=int, dest="conveyor__pwm__duty"
    )
    parser.add_argument(
        "--serial.baud-rate", default=9600, type=int, dest="serial_baud_rate"
    )
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
        dest="board__square__mm",
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


def tracking_loop(video_path: os.PathLike, parent_p: Connection, model="yolov8n.pt", ready_p=None):
    print("tracking_loop: importing yolo")
    from ultralytics import YOLO

    print("tracking_loop: imported yolo")

    print("tracking_loop: loading model")
    m = YOLO(model)
    print("tracking_loop: model loaded, starting video capture")
    capture = cv2.VideoCapture(video_path)
    print("tracking_loop: video capture created")

    ready = False

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
                        "class": box.cls.detach().cpu(),
                        "xyxy": list(p.detach().cpu()),
                    }
                )
        # print(f"sending results {len(ret)=}")
        parent_p.send(ret)
        if not ready:
            ready = True
            ready_p.send(True)
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


def decider_loop(pipe: Connection, port: str, baud_rate: int, conveyor_speed: int, shaker_speer: int):
    ser = serial.Serial(port, baud_rate, timeout=1)

    while True:
        coords = pipe.recv()
        while ser.in_waiting > 0:
            response = ser.read_until()
            print(f"decider_loop: got serial response: {response}")

        coords = [
            {
                "class": c["class"],
                "cog": [
                    (c["xyxy_transformed"][0][0] + c["xyxy_transformed"][1][0]) / 2,
                    (c["xyxy_transformed"][0][1] + c["xyxy_transformed"][1][1]) / 2,
                ],
            }
            for c in coords
            if int(c["class"]) != 2  # labels have class 2
        ]

        coords.sort(key=lambda c: c["cog"][1])  # sort by y axis of COG

        if len(coords) < 1:
            continue

        colour = int(
            coords[0]["class"]
        )  # TODO: get bottle closest to gate that has not passed gate

        print(f"decider_loop: {colour=}")

        cmd = f"c{conveyor_speed:04}\n".encode()
        print(cmd)
        ser.write(cmd)
        cmd = f"s{shaker_speed:04}\n".encode()
        print(cmd)
        ser.write(cmd)

        if colour == 0:
            print("g0020")
            ser.write("g0020\n".encode("utf-8"))
            ser.flush()
        else:
            print("g0050")
            ser.write("g0050\n".encode("utf-8"))
            ser.flush()


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
    (tracker_ready_p, tracker_ready_child_p) = Pipe()
    tracker_handle = Process(
        target=tracking_loop, args=(args.video_path, tracker_child_p, args.model_path, tracker_ready_child_p)
    )
    tracker_handle.start()

    print("Creating transformer loop")
    (xf_p, xf_child_p) = Pipe()
    xf_handle = Process(target=transformer_loop, args=(tracker_p, xf_child_p, M))
    xf_handle.start()

    print("Starting decidero loop")
    decider_handle = Process(
        target=decider_loop, args=(xf_p, args.serial_port, args.serial_baud_rate)
    )
    decider_handle.start()

    print("Waiting for tracker to start...")
    tracker_ready_p.recv() # tracker will send something when it is first ready. wait for it before staring shaking or conveying

    print("Creating serial connection")
    ser = serial.Serial(args.serial_port, args.serial_baud_rate, timeout=1)

    print(f"Starting conveyor ({args.conveyor__pwm__duty=})")
    cmd = f"c{int(args.conveyor__pwm__duty):04}\n".encode()
    print(f"SENDING: {cmd}")
    ser.write(cmd)
    ser.flush()
    while ser.in_waiting > 0:
        response = ser.read_until()
        print(f"got serial response: {response}")

    print(f"Starting shaker table ({args.shaker_table__pwm__duty=})")
    cmd = f"s{int(args.shaker_table__pwm__duty):04}\n".encode()
    print(f"SENDING: {cmd}")
    ser.write(cmd)
    ser.flush()
    while ser.in_waiting > 0:
        response = ser.read_until()
        print(f"got serial response: {response}")

    print("Closing setup serial connection")
    ser.close()

    while True:
        pass
        # if xf_p.poll():
        # recv = xf_p.recv()
        # print(f"xf_p.recv()={recv}")

    # if transformer_p.poll():
    #    recv = transformer_p.recv()
    #    annotated = recv["results"].plot()
    #    processed = recv["processed"]
    #    print(f"{processed=}")
    #    cv2.imshow("tracking", annotated)


if __name__ == "__main__":
    args = args()
    try:
        sys.exit(main(args))
    except:
        ser = serial.Serial(args.serial_port, args.serial_baud_rate, timeout=1)

        cmd = f"c{0:04}\n".encode()
        print(f"SENDING: {cmd}")
        ser.write(cmd)

        cmd = f"s{0:04}\n".encode()
        print(f"SENDING: {cmd}")
        ser.write(cmd)

        cmd = "g0020\n".encode("utf-8")
        print(f"SENDING: {cmd}")
        ser.write(cmd)
