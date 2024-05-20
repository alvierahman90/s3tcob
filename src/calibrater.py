import numpy as np
import cv2


class PaddingMillimetres:
    def __init__(self, left, top, right, bottom):
        self.left = left
        self.top = top
        self.right = right
        self.bottom = bottom


class Calibrater:
    def __init__(self, px_per_mm, square_mm, board_dims, padding):
        self.px_per_mm = px_per_mm
        self.square_mm = square_mm
        self.board_dims = board_dims
        self.padding = padding
        self.M = None

        self._update()

    def _update(self):
        square_px = self.px_per_mm * self.square_mm
        self.padding_px = 3 * square_px
        grid_x_px = square_px * (self.board_dims[0] - 1)
        grid_y_px = square_px * (self.board_dims[1] - 1)

        padding_left = self.padding * self.px_per_mm
        padding_top = self.padding * self.px_per_mm
        padding_right = self.padding * self.px_per_mm
        padding_bottom = self.padding * self.px_per_mm

        print(f"{self.board_dims[0]=} {self.board_dims[1]} {grid_x_px=} {grid_y_px=}")

        self.img_x_px = padding_left + padding_right + grid_x_px
        self.img_y_px = padding_top + padding_bottom + grid_y_px

        left = int(padding_left)
        right = int(left + grid_x_px)
        top = int(padding_top)
        bottom = int(top + grid_y_px)

        print(f"{self.img_y_px=} {self.img_x_px=} {left=} {right=}")

        self.pts2 = np.float32(
            [
                [left, top],  # top left
                [right, top],  # top right
                [left, bottom],  # bottom left
                [right, bottom],  # bottom right
            ]
        )

    def _pts1(self, corners):
        """
        returns the 4 outer corners of the chessboard

        corners: an array of corners that are sorted according to
            https://github.com/alvierahman90/MMME4085_Colour_Identification/tree/main/calibration#cv2findchessboardcorners-output
             (opencv.findChessboardCorners does this for you)
        """
        return np.float32(
            [
                corners[0],
                corners[self.board_dims[0] - 1],
                corners[-self.board_dims[0]],
                corners[-1],
            ]
        )

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

    def get_warped_image(self, image):
        """
        get warped image based on calibrated transformation matrix
        """

        (ok, M, corners) = self.get_calibration_transform(image)
        if not ok:
            return (False, None)

        # plot chess board points
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(image, corners, (11, 11), (-1, -1), criteria)
        cv2.drawChessboardCorners(image, self.board_dims, corners2)
        for pt in self._pts1(corners):
            cX, cY = pt[0][0], pt[0][1]
            center = (int(cX), int(cY))
            radius = 10
            colour = (0, 0, 255)
            image = cv2.circle(image, center, radius, colour, 3)

        image = cv2.warpPerspective(image, M, (self.img_x_px, self.img_y_px))
        return (True, image)
