# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""IP camera backend built on OpenCV."""

from __future__ import annotations

import contextlib
import threading
import time
from typing import TYPE_CHECKING, cast

import cv2
import numpy as np

from physicalai.capture.camera import Camera, ColorMode
from physicalai.capture.errors import CaptureError, CaptureTimeoutError, NotConnectedError
from physicalai.capture.frame import Frame

if TYPE_CHECKING:
    from physicalai.capture.discovery import DeviceInfo


_RGB_CHANNEL_COUNT = 3
_GRAY_DIMENSIONS = 2


class IPCamera(Camera):
    """IP camera / network stream backend.

    Args:
        device: OpenCV source identifier. This can be an integer index,
            RTSP URL, MJPEG-over-HTTP endpoint such as ``/stream``, or any
            other string accepted by :class:`cv2.VideoCapture`.
        width: Requested output width hint.
        height: Requested output height hint.
        fps: Requested frame-rate hint.
        color_mode: Output pixel format.
        backend: Optional OpenCV capture backend preference. When omitted,
            network sources default to ``CAP_FFMPEG`` and local indices use
            ``CAP_ANY``.
    """

    _POLL_INTERVAL_S = 0.001

    def __init__(
        self,
        *,
        device: int | str = 0,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        color_mode: ColorMode = ColorMode.RGB,
        backend: int | str | None = None,
    ) -> None:
        super().__init__(color_mode=color_mode)
        self._device = device
        self._width = width
        self._height = height
        self._fps = fps
        self._backend = backend
        self._capture: cv2.VideoCapture | None = None
        self._reader_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._condition = threading.Condition()
        self._connected = False
        self._sequence = -1
        self._delivered_sequence = -1
        self._latest_frame: np.ndarray | None = None
        self._latest_timestamp = 0.0
        self._reader_error: CaptureError | None = None

    def _normalize_device_input(self) -> int | str:
        if isinstance(self._device, str) and self._device.isdecimal():
            return int(self._device)
        return self._device

    def _resolve_backend(self) -> int:
        backend = self._backend
        if isinstance(backend, int):
            return backend

        any_backend = cast("int", getattr(cv2, "CAP_ANY", 0))
        ffmpeg_backend = cast("int", getattr(cv2, "CAP_FFMPEG", any_backend))
        gstreamer_backend = cast("int", getattr(cv2, "CAP_GSTREAMER", ffmpeg_backend))

        if isinstance(backend, str):
            normalized = backend.lower()
            if normalized == "any":
                return any_backend
            if normalized == "ffmpeg":
                return ffmpeg_backend
            if normalized == "gstreamer":
                return gstreamer_backend
            msg = f"Unknown backend {backend!r}. Use 'any', 'ffmpeg', or 'gstreamer'."
            raise ValueError(msg)

        source = self._normalize_device_input()
        if isinstance(source, str):
            return ffmpeg_backend
        return any_backend

    def _open_capture(self) -> cv2.VideoCapture:
        capture = cv2.VideoCapture(self._normalize_device_input(), self._resolve_backend())
        if not capture.isOpened():
            msg = f"Failed to open IP camera source {self._device!r}"
            raise CaptureError(msg)

        for prop, value in (
            (getattr(cv2, "CAP_PROP_BUFFERSIZE", None), 1),
            (getattr(cv2, "CAP_PROP_FRAME_WIDTH", None), self._width),
            (getattr(cv2, "CAP_PROP_FRAME_HEIGHT", None), self._height),
            (getattr(cv2, "CAP_PROP_FPS", None), self._fps),
        ):
            if prop is not None:
                with contextlib.suppress(Exception):
                    capture.set(prop, value)
        return capture

    def connect(self, timeout: float = 5.0) -> None:
        self.disconnect()
        self._stop_event = threading.Event()
        self._condition = threading.Condition()
        self._reader_error = None
        self._latest_frame = None
        self._latest_timestamp = 0.0
        self._sequence = -1
        self._delivered_sequence = -1

        try:
            self._capture = self._open_capture()
        except Exception:
            self._capture = None
            raise

        self._reader_thread = threading.Thread(target=self._reader_loop, name="ip-camera-reader", daemon=True)
        self._reader_thread.start()

        deadline = time.monotonic() + timeout
        reader_error: CaptureError | None = None
        timed_out = False
        with self._condition:
            while self._latest_frame is None and self._reader_error is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    timed_out = True
                    break
                self._condition.wait(timeout=remaining)

            if self._reader_error is not None:
                reader_error = self._reader_error

            if self._latest_frame is None:
                timed_out = True

            if reader_error is None and not timed_out:
                self._connected = True
                self._delivered_sequence = self._sequence
                return

        self._do_disconnect()
        if reader_error is not None:
            raise reader_error
        msg = f"Timed out waiting for first frame after {timeout}s"
        raise CaptureTimeoutError(msg)

    def _reader_loop(self) -> None:
        capture = self._capture
        if capture is None:
            return

        while not self._stop_event.is_set():
            ok, frame = capture.read()
            if not ok or frame is None:
                if self._stop_event.is_set():
                    break
                with self._condition:
                    self._reader_error = CaptureError(f"Failed to read frame from {self.device_id}")
                    self._connected = False
                    self._condition.notify_all()
                break

            converted = self._convert_color(frame)
            timestamp = time.monotonic()
            with self._condition:
                self._latest_frame = converted
                self._latest_timestamp = timestamp
                self._sequence += 1
                self._condition.notify_all()

    def _do_disconnect(self) -> None:
        self._connected = False
        self._stop_event.set()

        capture = self._capture
        self._capture = None
        if capture is not None:
            with contextlib.suppress(Exception):
                capture.release()

        thread = self._reader_thread
        self._reader_thread = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=0.5)

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def device_id(self) -> str:
        return str(self._device)

    def _ensure_connected(self) -> None:
        if self._reader_error is not None:
            raise self._reader_error
        if not self._connected or self._capture is None:
            raise NotConnectedError

    def _wait_for_frame(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._reader_error is None and self._sequence == self._delivered_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    msg = f"Timed out waiting for frame after {timeout}s"
                    raise CaptureTimeoutError(msg)
                self._condition.wait(timeout=remaining)

            if self._reader_error is not None:
                raise self._reader_error

    def _snapshot_latest(self) -> Frame:
        if self._latest_frame is None:
            msg = f"No frame has been captured yet from {self.device_id}"
            raise CaptureError(msg)

        self._delivered_sequence = self._sequence
        return Frame(
            data=self._latest_frame,
            timestamp=self._latest_timestamp,
            sequence=self._sequence,
        )

    def read(self, timeout: float = 2.0) -> Frame:
        self._ensure_connected()
        self._wait_for_frame(timeout)
        return self._snapshot_latest()

    def read_latest(self) -> Frame:
        self._ensure_connected()
        with self._condition:
            if self._reader_error is not None:
                raise self._reader_error
            return self._snapshot_latest()

    def _convert_color(self, frame: np.ndarray) -> np.ndarray:
        if self._color_mode == ColorMode.RGB:
            if frame.ndim == _RGB_CHANNEL_COUNT and frame.shape[2] >= _RGB_CHANNEL_COUNT:
                return np.ascontiguousarray(frame[:, :, :3][:, :, ::-1])
            return np.ascontiguousarray(frame)
        if self._color_mode == ColorMode.BGR:
            return np.ascontiguousarray(frame)
        if frame.ndim == _GRAY_DIMENSIONS:
            return np.ascontiguousarray(frame)
        return np.dot(frame[..., :3], [0.2989, 0.5870, 0.1140]).astype(np.uint8)

    @classmethod
    def discover(cls) -> list[DeviceInfo]:
        return []
