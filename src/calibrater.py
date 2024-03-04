import numpy as np
import cv2


class Calibrater:

    def __init__(self, px_per_mm, square_mm, board_dims):
        self.px_per_mm = px_per_mm
        self.square_mm = square_mm
        self.board_dims = board_dims
        self.M = None

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
        self.M = np.copy(M)

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

