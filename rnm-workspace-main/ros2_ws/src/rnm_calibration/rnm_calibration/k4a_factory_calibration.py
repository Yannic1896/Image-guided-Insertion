"""Extract Azure Kinect factory calibration through the k4a C SDK."""

from __future__ import annotations

import argparse
import ctypes
from ctypes import util as ctypes_util
from datetime import datetime, timezone
import json
import os
import sys

try:
    import yaml
except ImportError:
    yaml = None


K4A_RESULT_SUCCEEDED = 0
K4A_BUFFER_RESULT_SUCCEEDED = 0
K4A_BUFFER_RESULT_TOO_SMALL = 2

K4A_CALIBRATION_TYPE_DEPTH = 0
K4A_CALIBRATION_TYPE_COLOR = 1
K4A_CALIBRATION_TYPE_GYRO = 2
K4A_CALIBRATION_TYPE_ACCEL = 3
K4A_CALIBRATION_TYPE_NUM = 4

DEPTH_MODES = {
    "OFF": 0,
    "NFOV_2X2BINNED": 1,
    "NFOV_UNBINNED": 2,
    "WFOV_2X2BINNED": 3,
    "WFOV_UNBINNED": 4,
    "PASSIVE_IR": 5,
}

COLOR_RESOLUTIONS = {
    "OFF": 0,
    "720P": 1,
    "1080P": 2,
    "1440P": 3,
    "1536P": 4,
    "2160P": 5,
    "3072P": 6,
}

CALIBRATION_MODEL_NAMES = {
    0: "UNKNOWN",
    1: "THETA",
    2: "POLYNOMIAL_3K",
    3: "RATIONAL_6KT",
    4: "BROWN_CONRADY",
}

CALIBRATION_TYPE_NAMES = {
    K4A_CALIBRATION_TYPE_DEPTH: "depth",
    K4A_CALIBRATION_TYPE_COLOR: "color",
    K4A_CALIBRATION_TYPE_GYRO: "gyro",
    K4A_CALIBRATION_TYPE_ACCEL: "accel",
}


class K4ACalibrationExtrinsics(ctypes.Structure):
    """ctypes mirror of k4a_calibration_extrinsics_t."""

    _fields_ = [
        ("rotation", ctypes.c_float * 9),
        ("translation", ctypes.c_float * 3),
    ]


class K4ACalibrationIntrinsicParamStruct(ctypes.Structure):
    """Named view of k4a_calibration_intrinsic_parameters_t."""

    _fields_ = [
        ("cx", ctypes.c_float),
        ("cy", ctypes.c_float),
        ("fx", ctypes.c_float),
        ("fy", ctypes.c_float),
        ("k1", ctypes.c_float),
        ("k2", ctypes.c_float),
        ("k3", ctypes.c_float),
        ("k4", ctypes.c_float),
        ("k5", ctypes.c_float),
        ("k6", ctypes.c_float),
        ("codx", ctypes.c_float),
        ("cody", ctypes.c_float),
        ("p2", ctypes.c_float),
        ("p1", ctypes.c_float),
        ("metric_radius", ctypes.c_float),
    ]


class K4ACalibrationIntrinsicParameters(ctypes.Union):
    """ctypes mirror of k4a_calibration_intrinsic_parameters_t."""

    _fields_ = [
        ("param", K4ACalibrationIntrinsicParamStruct),
        ("v", ctypes.c_float * 15),
    ]


class K4ACalibrationIntrinsics(ctypes.Structure):
    """ctypes mirror of k4a_calibration_intrinsics_t."""

    _fields_ = [
        ("type", ctypes.c_int),
        ("parameter_count", ctypes.c_uint),
        ("parameters", K4ACalibrationIntrinsicParameters),
    ]


class K4ACalibrationCamera(ctypes.Structure):
    """ctypes mirror of k4a_calibration_camera_t."""

    _fields_ = [
        ("extrinsics", K4ACalibrationExtrinsics),
        ("intrinsics", K4ACalibrationIntrinsics),
        ("resolution_width", ctypes.c_int),
        ("resolution_height", ctypes.c_int),
        ("metric_radius", ctypes.c_float),
    ]


class K4ACalibration(ctypes.Structure):
    """ctypes mirror of k4a_calibration_t."""

    _fields_ = [
        ("depth_camera_calibration", K4ACalibrationCamera),
        ("color_camera_calibration", K4ACalibrationCamera),
        (
            "extrinsics",
            (K4ACalibrationExtrinsics * K4A_CALIBRATION_TYPE_NUM)
            * K4A_CALIBRATION_TYPE_NUM,
        ),
        ("depth_mode", ctypes.c_int),
        ("color_resolution", ctypes.c_int),
    ]


class K4ASdk:
    """Small ctypes binding for the k4a device calibration API."""

    def __init__(self, library_path: str | None = None) -> None:
        self.library_path = library_path
        self._library = self._load_library(library_path)
        self._configure_functions()

    @property
    def resolved_library_path(self) -> str:
        """Return the resolved dynamic library path if available."""
        return str(getattr(self._library, "_name", self.library_path))

    def installed_count(self) -> int:
        """Return the number of Azure Kinect devices detected by the SDK."""
        return int(self._library.k4a_device_get_installed_count())

    def open_device(self, index: int):
        """Open an Azure Kinect device and return its opaque handle."""
        handle = ctypes.c_void_p()
        result = self._library.k4a_device_open(
            ctypes.c_uint32(index),
            ctypes.byref(handle),
        )
        if result != K4A_RESULT_SUCCEEDED:
            raise RuntimeError(f"k4a_device_open({index}) failed.")
        return handle

    def close_device(self, handle) -> None:
        """Close an Azure Kinect device handle."""
        if handle:
            self._library.k4a_device_close(handle)

    def get_serial_number(self, handle) -> str | None:
        """Read the serial number for a device handle."""
        size = ctypes.c_size_t(0)
        result = self._library.k4a_device_get_serialnum(
            handle,
            None,
            ctypes.byref(size),
        )
        if result != K4A_BUFFER_RESULT_TOO_SMALL:
            return None

        buffer = ctypes.create_string_buffer(size.value)
        result = self._library.k4a_device_get_serialnum(
            handle,
            buffer,
            ctypes.byref(size),
        )
        if result != K4A_BUFFER_RESULT_SUCCEEDED:
            return None
        return buffer.value.decode("utf-8", errors="replace")

    def get_calibration(
        self,
        handle,
        depth_mode: int,
        color_resolution: int,
    ) -> K4ACalibration:
        """Read factory camera calibration for a mode/resolution pair."""
        calibration = K4ACalibration()
        result = self._library.k4a_device_get_calibration(
            handle,
            depth_mode,
            color_resolution,
            ctypes.byref(calibration),
        )
        if result != K4A_RESULT_SUCCEEDED:
            raise RuntimeError(
                "k4a_device_get_calibration failed for "
                f"depth_mode={depth_mode}, "
                f"color_resolution={color_resolution}."
            )
        return calibration

    def _load_library(self, library_path: str | None):
        candidates = []
        if library_path:
            candidates.append(library_path)
        env_path = os.environ.get("K4A_LIBRARY")
        if env_path:
            candidates.append(env_path)
        for name in ("k4a", "k4a1.4"):
            found = ctypes_util.find_library(name)
            if found:
                candidates.append(found)
        candidates.extend([
            "libk4a.so",
            "libk4a.so.1.4",
            "/usr/lib/x86_64-linux-gnu/libk4a.so",
            "/usr/lib/x86_64-linux-gnu/libk4a.so.1.4",
            "/usr/local/lib/libk4a.so",
        ])

        errors = []
        for candidate in _dedupe(candidates):
            try:
                return ctypes.CDLL(candidate)
            except OSError as exc:
                errors.append(f"{candidate}: {exc}")
        joined = "\n  ".join(errors) if errors else "no candidates found"
        raise RuntimeError(
            "Could not load the Azure Kinect Sensor SDK library.\n"
            "Install libk4a in the container or pass --library.\n"
            f"Tried:\n  {joined}"
        )

    def _configure_functions(self) -> None:
        self._library.k4a_device_get_installed_count.argtypes = []
        self._library.k4a_device_get_installed_count.restype = ctypes.c_uint32

        self._library.k4a_device_open.argtypes = [
            ctypes.c_uint32,
            ctypes.POINTER(ctypes.c_void_p),
        ]
        self._library.k4a_device_open.restype = ctypes.c_int

        self._library.k4a_device_close.argtypes = [ctypes.c_void_p]
        self._library.k4a_device_close.restype = None

        self._library.k4a_device_get_serialnum.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.POINTER(ctypes.c_size_t),
        ]
        self._library.k4a_device_get_serialnum.restype = ctypes.c_int

        self._library.k4a_device_get_calibration.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.POINTER(K4ACalibration),
        ]
        self._library.k4a_device_get_calibration.restype = ctypes.c_int


def build_calibration_document(
    calibration: K4ACalibration,
    sdk: K4ASdk,
    args,
    serial_number: str | None,
):
    """Convert k4a_calibration_t into a publisher-compatible document."""
    depth_to_color = _extrinsics(calibration, "depth", "color")
    color_to_depth = _extrinsics(calibration, "color", "depth")

    return {
        "calibration": {
            "version": 1,
            "source": "azure_kinect_sensor_sdk_factory",
            "calibrated_at": datetime.now(timezone.utc).isoformat(),
            "sdk_library": sdk.resolved_library_path,
            "device_index": int(args.device_index),
            "serial_number": serial_number,
            "depth_mode": args.depth_mode,
            "color_resolution": args.color_resolution,
            "translation_unit": "m",
        },
        "rgb_camera": _camera_section(
            "azure_kinect_rgb",
            args.rgb_frame_id,
            calibration.color_camera_calibration,
        ),
        "depth_camera": _camera_section(
            "azure_kinect_depth",
            args.depth_frame_id,
            calibration.depth_camera_calibration,
        ),
        "extrinsics": {
            "parent_frame_id": args.rgb_frame_id,
            "child_frame_id": args.depth_frame_id,
            "rotation_depth_to_rgb": _matrix_field(
                depth_to_color["rotation"]
            ),
            "translation_depth_to_rgb": _vector_field(
                _mm_to_m(depth_to_color["translation"])
            ),
            "rotation_rgb_to_depth": _matrix_field(
                color_to_depth["rotation"]
            ),
            "translation_rgb_to_depth": _vector_field(
                _mm_to_m(color_to_depth["translation"])
            ),
        },
        "k4a_raw_extrinsics": _all_extrinsics(calibration),
    }


def write_document(document, output_path: str, output_format: str) -> str:
    """Write the calibration document to JSON or YAML."""
    if output_format == "yaml" and yaml is None:
        raise RuntimeError(
            "YAML output requires PyYAML. Install python3-yaml, "
            "or rerun with --format json."
        )

    resolved_path = os.path.abspath(
        os.path.expandvars(os.path.expanduser(output_path))
    )
    os.makedirs(os.path.dirname(resolved_path), exist_ok=True)

    with open(resolved_path, "w", encoding="utf-8") as file_obj:
        if output_format == "json":
            json.dump(document, file_obj, indent=2)
            file_obj.write("\n")
        else:
            yaml.safe_dump(document, file_obj, sort_keys=False)
    return resolved_path


def parse_args(argv=None):
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Extract Azure Kinect factory intrinsics and extrinsics through "
            "the k4a Sensor SDK."
        )
    )
    parser.add_argument(
        "--check-sdk",
        action="store_true",
        help="Only check whether libk4a can be loaded and list device count.",
    )
    parser.add_argument(
        "--library",
        default=None,
        help="Optional path to libk4a.so if it is not on the linker path.",
    )
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument(
        "--depth-mode",
        choices=sorted(DEPTH_MODES),
        default="NFOV_UNBINNED",
    )
    parser.add_argument(
        "--color-resolution",
        choices=sorted(COLOR_RESOLUTIONS),
        default="720P",
    )
    parser.add_argument(
        "--rgb-frame-id",
        default="rgb_camera_optical_frame",
    )
    parser.add_argument(
        "--depth-frame-id",
        default="depth_camera_optical_frame",
    )
    parser.add_argument(
        "--format",
        choices=("yaml", "json"),
        default="yaml",
        help="Output serialization format.",
    )
    parser.add_argument(
        "--output",
        default="~/.ros/rnm_calibration/azure_kinect_factory_calibration.yaml",
        help="Output file path.",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    """Run the k4a factory calibration extractor."""
    args = parse_args(argv)
    try:
        sdk = K4ASdk(args.library)
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 2

    device_count = sdk.installed_count()
    if args.check_sdk:
        print(f"Loaded {sdk.resolved_library_path}")
        print(f"Detected Azure Kinect devices: {device_count}")
        return 0

    if device_count <= args.device_index:
        print(
            "No Azure Kinect device is available at index "
            f"{args.device_index}; detected {device_count}.",
            file=sys.stderr,
        )
        return 3

    handle = None
    try:
        handle = sdk.open_device(args.device_index)
        serial_number = sdk.get_serial_number(handle)
        calibration = sdk.get_calibration(
            handle,
            DEPTH_MODES[args.depth_mode],
            COLOR_RESOLUTIONS[args.color_resolution],
        )
        document = build_calibration_document(
            calibration,
            sdk,
            args,
            serial_number,
        )
        output_path = write_document(document, args.output, args.format)
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 4
    finally:
        sdk.close_device(handle)

    print(f"Wrote Azure Kinect factory calibration to {output_path}")
    return 0


def _camera_section(name: str, frame_id: str, camera: K4ACalibrationCamera):
    intrinsics = camera.intrinsics
    params = intrinsics.parameters.param
    camera_matrix = [
        [float(params.fx), 0.0, float(params.cx)],
        [0.0, float(params.fy), float(params.cy)],
        [0.0, 0.0, 1.0],
    ]
    projection_matrix = [
        [float(params.fx), 0.0, float(params.cx), 0.0],
        [0.0, float(params.fy), float(params.cy), 0.0],
        [0.0, 0.0, 1.0, 0.0],
    ]
    distortion = [
        float(params.k1),
        float(params.k2),
        float(params.p1),
        float(params.p2),
        float(params.k3),
        float(params.k4),
        float(params.k5),
        float(params.k6),
    ]

    return {
        "camera_name": name,
        "frame_id": frame_id,
        "image_width": int(camera.resolution_width),
        "image_height": int(camera.resolution_height),
        "distortion_model": "rational_polynomial",
        "camera_matrix": _matrix_field(camera_matrix),
        "distortion_coefficients": _matrix_field([distortion]),
        "rectification_matrix": _matrix_field(_identity_3x3()),
        "projection_matrix": _matrix_field(projection_matrix),
        "k4a_intrinsics": {
            "model": CALIBRATION_MODEL_NAMES.get(
                int(intrinsics.type),
                f"UNKNOWN_{int(intrinsics.type)}",
            ),
            "parameter_count": int(intrinsics.parameter_count),
            "parameters": {
                "cx": float(params.cx),
                "cy": float(params.cy),
                "fx": float(params.fx),
                "fy": float(params.fy),
                "k1": float(params.k1),
                "k2": float(params.k2),
                "k3": float(params.k3),
                "k4": float(params.k4),
                "k5": float(params.k5),
                "k6": float(params.k6),
                "codx": float(params.codx),
                "cody": float(params.cody),
                "p1": float(params.p1),
                "p2": float(params.p2),
                "metric_radius": float(params.metric_radius),
            },
            "metric_radius": float(camera.metric_radius),
        },
    }


def _extrinsics(calibration: K4ACalibration, source: str, target: str):
    source_index = _calibration_type_index(source)
    target_index = _calibration_type_index(target)
    extrinsics = calibration.extrinsics[source_index][target_index]
    return {
        "rotation": _rotation_matrix(extrinsics.rotation),
        "translation": [float(value) for value in extrinsics.translation],
    }


def _all_extrinsics(calibration: K4ACalibration):
    all_values = {}
    for source_index, source_name in CALIBRATION_TYPE_NAMES.items():
        for target_index, target_name in CALIBRATION_TYPE_NAMES.items():
            extrinsics = calibration.extrinsics[source_index][target_index]
            key = f"{source_name}_to_{target_name}"
            all_values[key] = {
                "rotation": _matrix_field(
                    _rotation_matrix(extrinsics.rotation)
                ),
                "translation_mm": _vector_field(extrinsics.translation),
            }
    return all_values


def _calibration_type_index(name: str) -> int:
    for index, type_name in CALIBRATION_TYPE_NAMES.items():
        if type_name == name:
            return index
    raise ValueError(f"Unknown k4a calibration type: {name}")


def _rotation_matrix(values):
    flat = [float(value) for value in values]
    return [flat[0:3], flat[3:6], flat[6:9]]


def _matrix_field(matrix):
    rows = len(matrix)
    cols = len(matrix[0]) if rows else 0
    return {
        "rows": rows,
        "cols": cols,
        "data": [
            float(matrix[row][col])
            for row in range(rows)
            for col in range(cols)
        ],
    }


def _vector_field(values):
    data = [float(value) for value in values]
    return {
        "rows": len(data),
        "cols": 1,
        "data": data,
    }


def _identity_3x3():
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _mm_to_m(values):
    return [float(value) / 1000.0 for value in values]


def _dedupe(values):
    seen = set()
    for value in values:
        if value and value not in seen:
            seen.add(value)
            yield value


if __name__ == "__main__":
    raise SystemExit(main())
