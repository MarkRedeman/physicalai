# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: D103, PLR2004, S101

"""Tests for IPCamera."""

from __future__ import annotations

import importlib
import queue
import sys
import time
from typing import TYPE_CHECKING
from unittest import mock

import numpy as np
import pytest

from physicalai.capture.errors import CaptureTimeoutError, NotConnectedError
from physicalai.capture.frame import Frame

if TYPE_CHECKING:
    from collections.abc import Callable


def _wait_for(predicate: Callable[[], bool], *, timeout: float = 1.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    msg = "timed out waiting for condition"
    raise AssertionError(msg)


@pytest.fixture
def ip_cls() -> tuple[type, queue.Queue[np.ndarray | None], list[object], object]:
    """Inject a mocked cv2 module and reload IPCamera.

    Yields:
        Tuple of IPCamera class, frame queue, capture instances, and mocked cv2.
    """
    frame_queue: queue.Queue[np.ndarray | None] = queue.Queue()
    capture_instances: list[object] = []

    class FakeVideoCapture:
        def __init__(self, source: int | str, api_preference: int = 0) -> None:
            self.source = source
            self.api_preference = api_preference
            self.props: dict[int, object] = {}
            self.closed = False
            self.frame_count = 0
            capture_instances.append(self)

        def isOpened(self) -> bool:  # noqa: N802
            return not self.closed

        def set(self, prop: int, value: object) -> bool:
            self.props[prop] = value
            return True

        def read(self) -> tuple[bool, np.ndarray | None]:
            if self.closed and frame_queue.empty():
                return False, None
            item = frame_queue.get()
            if item is None:
                return False, None
            self.frame_count += 1
            return True, item

        def release(self) -> None:
            self.closed = True
            frame_queue.put(None)

    mock_cv2 = mock.MagicMock()
    mock_cv2.VideoCapture = FakeVideoCapture
    mock_cv2.CAP_ANY = 0
    mock_cv2.CAP_FFMPEG = 1900
    mock_cv2.CAP_GSTREAMER = 1800
    mock_cv2.CAP_PROP_BUFFERSIZE = 38
    mock_cv2.CAP_PROP_FRAME_WIDTH = 3
    mock_cv2.CAP_PROP_FRAME_HEIGHT = 4
    mock_cv2.CAP_PROP_FPS = 5

    sys.modules["cv2"] = mock_cv2
    sys.modules.pop("physicalai.capture.cameras.ip._camera", None)

    module = importlib.import_module("physicalai.capture.cameras.ip._camera")

    yield module.IPCamera, frame_queue, capture_instances, mock_cv2

    sys.modules.pop("cv2", None)
    sys.modules.pop("physicalai.capture.cameras.ip._camera", None)


def test_connect_uses_ffmpeg_for_network_sources(ip_cls: tuple) -> None:
    camera_cls, frame_queue, capture_instances, mock_cv2 = ip_cls
    frame_queue.put(np.array([[[1, 2, 3]]], dtype=np.uint8))

    cam = camera_cls(device="http://camera.local/stream")
    cam.connect()

    assert cam.is_connected
    assert capture_instances[0].source == "http://camera.local/stream"
    assert capture_instances[0].api_preference == mock_cv2.CAP_FFMPEG
    cam.disconnect()


def test_read_latest_returns_newest_frame_and_drops_old_frames(ip_cls: tuple) -> None:
    camera_cls, frame_queue, capture_instances, _ = ip_cls
    frame0 = np.array([[[1, 2, 3]]], dtype=np.uint8)
    frame1 = np.array([[[4, 5, 6]]], dtype=np.uint8)
    frame2 = np.array([[[7, 8, 9]]], dtype=np.uint8)
    frame3 = np.array([[[10, 11, 12]]], dtype=np.uint8)

    frame_queue.put(frame0)
    cam = camera_cls(device="rtsp://camera/stream")
    cam.connect()

    frame_queue.put(frame1)
    frame_queue.put(frame2)
    _wait_for(lambda: capture_instances[0].frame_count >= 3)

    latest = cam.read_latest()
    assert isinstance(latest, Frame)
    assert latest.sequence == 2
    assert latest.data[0, 0].tolist() == [9, 8, 7]

    frame_queue.put(frame3)
    next_frame = cam.read(timeout=1.0)
    assert next_frame.sequence == 3
    assert next_frame.data[0, 0].tolist() == [12, 11, 10]
    cam.disconnect()


def test_read_before_connect_raises(ip_cls: tuple) -> None:
    camera_cls, *_ = ip_cls
    cam = camera_cls(device="rtsp://camera/stream")
    with pytest.raises(NotConnectedError):
        cam.read()


def test_read_latest_before_connect_raises(ip_cls: tuple) -> None:
    camera_cls, *_ = ip_cls
    cam = camera_cls(device="rtsp://camera/stream")
    with pytest.raises(NotConnectedError):
        cam.read_latest()


def test_connect_timeout_raises(ip_cls: tuple) -> None:
    camera_cls, *_ = ip_cls
    cam = camera_cls(device="rtsp://camera/stream")
    with pytest.raises(CaptureTimeoutError):
        cam.connect(timeout=0.05)
