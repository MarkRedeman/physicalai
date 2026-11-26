from __future__ import annotations

import math

import can
import pytest

from physicalai_openarm_plugin.damiao import BUS_RECOVERY_GRACE, DamiaoSocketCAN


def test_mit_position_round_trip_and_state_decode() -> None:
    transport = DamiaoSocketCAN("can0", {"joint_1": (1, 17, "dm8009")})
    frame = transport._encode_mit("dm8009", 240.0, 5.0, 45.0)  # noqa: SLF001
    assert len(frame) == 8

    state = transport._decode_state(  # noqa: SLF001
        bytes((0, 0x80, 0x00, 0x80, 0x08, 0x00, 30, 31)),
        "dm8009",
    )
    assert math.isclose(state.position, 0.0, abs_tol=0.1)
    assert state.temp_mos == 30.0
    assert state.temp_rotor == 31.0


class _FakeBus:
    """Answers refresh requests for motor 1 until ``down`` simulates a bus-off."""

    def __init__(self) -> None:
        self.down = False
        self.pending: list[can.Message] = []

    def send(self, message: can.Message) -> None:
        if self.down:
            msg = "bus-off"
            raise can.CanOperationError(msg)
        self.pending.append(can.Message(arbitration_id=17, data=bytes((1, 0x80, 0, 0x80, 0x08, 0, 30, 31))))

    def recv(self, timeout: float) -> can.Message | None:
        return self.pending.pop(0) if self.pending else None


def test_read_states_rides_out_a_short_bus_off() -> None:
    transport = DamiaoSocketCAN("can0", {"joint_1": (1, 17, "dm8009")})
    bus = _FakeBus()
    transport._bus = bus  # noqa: SLF001

    bus.down = True
    with pytest.raises(can.CanOperationError):  # no healthy poll yet, e.g. during connect
        transport.read_states()

    bus.down = False
    healthy = transport.read_states()
    bus.down = True
    assert transport.read_states() == healthy  # cached during the grace window
    transport.send_positions({"joint_1": (10.0, 1.0, 0.0)})  # dropped tick, no raise

    transport._healthy_at -= BUS_RECOVERY_GRACE  # noqa: SLF001
    with pytest.raises(can.CanOperationError):
        transport.read_states()
