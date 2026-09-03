"""Helpers for wiring Studio robots onto the physicalai transport."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from uuid import UUID


def shared_robot_name(robot_id: UUID) -> str:
    """Return the transport-safe ``SharedRobot`` name for a Studio robot."""
    return str(robot_id)
