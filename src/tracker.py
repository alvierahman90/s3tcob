import numpy as np
import cv2
from multiprocessing import Queue, Process, Pipe
from multiprocessing.connection import Connection
from ultralytics import YOLO


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

    def loop(self):
        """
        Start a tracking loop.
        """
        while True:
            image = self.queue.get()
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
        handle = Process(target=self.loop, args=())
        handle.start()

        return (handle, parent_p)

