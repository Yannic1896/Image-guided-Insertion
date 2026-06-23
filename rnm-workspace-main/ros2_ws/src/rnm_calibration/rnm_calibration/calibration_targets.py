"""OpenCV calibration target detection helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class TargetDetection:
    """Detected 2D target points and their corresponding 3D object points."""

    image_points: np.ndarray
    object_points: np.ndarray
    image_size: tuple[int, int]
    ids: Optional[np.ndarray] = None


class CheckerboardTarget:
    """Detect a planar checkerboard with OpenCV."""

    def __init__(self, columns: int, rows: int, square_size_m: float) -> None:
        if columns < 2 or rows < 2:
            raise ValueError(
                "Checkerboard rows and columns must both be >= 2."
            )
        if square_size_m <= 0.0:
            raise ValueError("Checkerboard square size must be positive.")

        self._columns = columns
        self._rows = rows
        self._pattern_size = (columns, rows)
        self._object_points = self._make_object_points(square_size_m)

    def detect(self, gray_image: np.ndarray) -> Optional[TargetDetection]:
        """Find checkerboard corners in a grayscale image."""
        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH
            | cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_FAST_CHECK
        )
        found, corners = cv2.findChessboardCorners(
            gray_image,
            self._pattern_size,
            flags,
        )
        if not found:
            return None

        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )
        refined = cv2.cornerSubPix(
            gray_image,
            corners,
            (11, 11),
            (-1, -1),
            criteria,
        )
        return TargetDetection(
            image_points=refined.reshape(-1, 2).astype(np.float32),
            object_points=self._object_points.copy(),
            image_size=_image_size(gray_image),
        )

    def _make_object_points(self, square_size_m: float) -> np.ndarray:
        points = np.zeros((self._rows * self._columns, 3), dtype=np.float32)
        grid = np.mgrid[0:self._columns, 0:self._rows].T.reshape(-1, 2)
        points[:, :2] = grid * float(square_size_m)
        return points


class CharucoTarget:
    """Detect a planar ChArUco target with OpenCV ArUco support."""

    def __init__(
        self,
        squares_x: int,
        squares_y: int,
        square_size_m: float,
        marker_size_m: float,
        dictionary_name: str,
        min_corners: int,
    ) -> None:
        if squares_x < 2 or squares_y < 2:
            raise ValueError("ChArUco squares_x and squares_y must be >= 2.")
        if square_size_m <= 0.0 or marker_size_m <= 0.0:
            raise ValueError(
                "ChArUco square and marker sizes must be positive."
            )
        if marker_size_m >= square_size_m:
            raise ValueError(
                "ChArUco marker size must be smaller than square size."
            )

        self._aruco = _aruco_module()
        self._dictionary = _aruco_dictionary(self._aruco, dictionary_name)
        self._board = _create_charuco_board(
            self._aruco,
            squares_x,
            squares_y,
            square_size_m,
            marker_size_m,
            self._dictionary,
        )
        self._object_points = _charuco_object_points(self._board)
        self._min_corners = int(min_corners)

    def detect(self, gray_image: np.ndarray) -> Optional[TargetDetection]:
        """Find ChArUco corners in a grayscale image."""
        marker_corners, marker_ids, _ = self._aruco.detectMarkers(
            gray_image,
            self._dictionary,
        )
        if marker_ids is None or len(marker_ids) == 0:
            return None

        interpolation = self._aruco.interpolateCornersCharuco(
            marker_corners,
            marker_ids,
            gray_image,
            self._board,
        )
        count, charuco_corners, charuco_ids = interpolation
        if (
            charuco_corners is None
            or charuco_ids is None
            or int(count) < self._min_corners
        ):
            return None

        ids = charuco_ids.reshape(-1).astype(np.int32)
        image_points = charuco_corners.reshape(-1, 2).astype(np.float32)
        object_points = self._object_points[ids].astype(np.float32)
        return TargetDetection(
            image_points=image_points,
            object_points=object_points,
            image_size=_image_size(gray_image),
            ids=ids,
        )


def make_target(
    target_type: str,
    checkerboard_columns: int,
    checkerboard_rows: int,
    checkerboard_square_size_m: float,
    charuco_squares_x: int,
    charuco_squares_y: int,
    charuco_square_size_m: float,
    charuco_marker_size_m: float,
    charuco_dictionary: str,
    charuco_min_corners: int,
):
    """Create a calibration target detector from ROS parameters."""
    normalized = target_type.strip().lower()
    if normalized == "checkerboard":
        return CheckerboardTarget(
            checkerboard_columns,
            checkerboard_rows,
            checkerboard_square_size_m,
        )
    if normalized == "charuco":
        return CharucoTarget(
            charuco_squares_x,
            charuco_squares_y,
            charuco_square_size_m,
            charuco_marker_size_m,
            charuco_dictionary,
            charuco_min_corners,
        )
    raise ValueError("target_type must be 'checkerboard' or 'charuco'.")


def image_to_grayscale(image: np.ndarray) -> np.ndarray:
    """Convert a ROS image payload from cv_bridge into uint8 grayscale."""
    if image.ndim == 3:
        if image.shape[2] == 4:
            return cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
        if image.shape[2] == 3:
            return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = image[:, :, 0]

    if image.dtype == np.uint8:
        return image

    numeric = np.asarray(image, dtype=np.float32)
    valid = numeric[np.isfinite(numeric)]
    valid = valid[valid > 0.0]
    if valid.size == 0:
        return np.zeros(numeric.shape[:2], dtype=np.uint8)

    low, high = np.percentile(valid, [2.0, 98.0])
    if high <= low:
        high = low + 1.0
    scaled = (numeric - low) * (255.0 / (high - low))
    scaled = np.clip(scaled, 0.0, 255.0)
    return scaled.astype(np.uint8)


def paired_detection_points(first: TargetDetection, second: TargetDetection):
    """Return object and image points that are common to two detections."""
    if first.ids is None or second.ids is None:
        if len(first.image_points) != len(second.image_points):
            return None
        return (
            first.object_points.astype(np.float32),
            first.image_points.astype(np.float32),
            second.image_points.astype(np.float32),
        )

    first_by_id = {
        int(point_id): index
        for index, point_id in enumerate(first.ids.reshape(-1))
    }
    second_by_id = {
        int(point_id): index
        for index, point_id in enumerate(second.ids.reshape(-1))
    }
    common_ids = sorted(set(first_by_id).intersection(second_by_id))
    if not common_ids:
        return None

    object_points = []
    first_points = []
    second_points = []
    for point_id in common_ids:
        first_index = first_by_id[point_id]
        second_index = second_by_id[point_id]
        object_points.append(first.object_points[first_index])
        first_points.append(first.image_points[first_index])
        second_points.append(second.image_points[second_index])

    return (
        np.asarray(object_points, dtype=np.float32),
        np.asarray(first_points, dtype=np.float32),
        np.asarray(second_points, dtype=np.float32),
    )


def _image_size(image: np.ndarray) -> tuple[int, int]:
    return int(image.shape[1]), int(image.shape[0])


def _aruco_module():
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "OpenCV was built without cv2.aruco. Install OpenCV contrib "
            "support "
            "before using target_type='charuco'."
        )
    return cv2.aruco


def _aruco_dictionary(aruco, dictionary_name: str):
    if not hasattr(aruco, dictionary_name):
        raise ValueError(f"Unknown ArUco dictionary: {dictionary_name}")
    dictionary_id = getattr(aruco, dictionary_name)
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(dictionary_id)
    return aruco.Dictionary_get(dictionary_id)


def _create_charuco_board(
    aruco,
    squares_x: int,
    squares_y: int,
    square_size_m: float,
    marker_size_m: float,
    dictionary,
):
    if hasattr(aruco, "CharucoBoard_create"):
        return aruco.CharucoBoard_create(
            squares_x,
            squares_y,
            square_size_m,
            marker_size_m,
            dictionary,
        )
    return aruco.CharucoBoard(
        (squares_x, squares_y),
        square_size_m,
        marker_size_m,
        dictionary,
    )


def _charuco_object_points(board) -> np.ndarray:
    if hasattr(board, "getChessboardCorners"):
        points = board.getChessboardCorners()
    else:
        points = board.chessboardCorners
    return np.asarray(points, dtype=np.float32).reshape(-1, 3)
