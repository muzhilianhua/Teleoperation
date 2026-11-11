"""PowerMonitor: live windows and trip records from telemetry."""

from __future__ import annotations

from teleop_host.power import PowerMonitor
from teleop_host.protocol.messages import Event, EventCode, JointFlag, Telemetry, TelemetryJoint
from teleop_host.sim.model import HandModel

FRESH = JointFlag.SERVO_FRESH
HLS = JointFlag.SERVO_FRESH | JointFlag.HAS_CURRENT


def telemetry(volts: list[float], current_ma: int = 0, load_pct: float = 0.0, armed: int = 0) -> Telemetry:
    joints = []
    for i, v in enumerate(volts):
        hls = i % 2 == 1  # like the hand: even indices SCS, odd HLS
        joints.append(
            TelemetryJoint(
                flags=int(HLS if hls else FRESH),
                voltage_deci_v=round(v * 10),
                current_ma=current_ma if hls else 0,
                load_deci_pct=round(load_pct * 10),
            )
        )
    return Telemetry(armed_count=armed, joints=joints)


class Clock:
    def __init__(self) -> None:
        self.t = 100.0

    def __call__(self) -> float:
        return self.t


def monitor(clock: Clock) -> PowerMonitor:
    info = HandModel().device_info()
    return PowerMonitor(lambda hand: info, clock)


def test_window_tracks_sag_and_supply_estimate() -> None:
    clock = Clock()
    pm = monitor(clock)
    pm.on_message("right", telemetry([6.0] * 9, current_ma=500, load_pct=20, armed=9))
    clock.t += 0.1
    sag = [6.0] * 9
    sag[8] = 4.6  # joint 9 at the end of the chain
    pm.on_message("right", telemetry(sag, current_ma=3000, load_pct=80, armed=9))
    clock.t += 0.1
    pm.on_message("right", telemetry([5.9] * 9, current_ma=1000, load_pct=10, armed=9))
    w = pm.summary()["right"]
    assert w.min_v == 4.6 and w.min_v_joint == 9
    assert w.min_median_v == 5.9
    # 4 HLS servos (odd indices of 9) x 3 A x 80 % duty
    assert abs(w.peak_supply_a - 4 * 3.0 * 0.8) < 1e-6
    assert abs(w.peak_phase_a - 12.0) < 1e-6
    assert abs(w.now_supply_a - 4 * 1.0 * 0.1) < 1e-6 and w.armed == 9


def test_trip_record_covers_both_hands_before_the_fault() -> None:
    clock = Clock()
    pm = monitor(clock)
    trips = []
    pm.on_trip(trips.append)
    pm.on_message("left", telemetry([5.2] * 9, current_ma=2500, load_pct=90, armed=9))
    pm.on_message("right", telemetry([5.3] * 9, current_ma=2000, load_pct=70, armed=9))
    clock.t += 0.05
    supply_voltage = 8  # teleop::Fault::SupplyVoltage
    pm.on_message("right", Event(EventCode.DISARMED, 2, 0, 0, (supply_voltage, 1, 0, 0), "DISARM ALL: supply voltage"))
    assert len(trips) == 1 and pm.trips[0] is trips[0]
    record = trips[0]
    assert record.hand == "right" and record.fault == "supply voltage" and record.joint == 0
    assert record.hands["left"].min_median_v == 5.2 and record.hands["right"].min_median_v == 5.3
    assert record.hands["left"].peak_supply_a > record.hands["right"].peak_supply_a


def test_operator_disarm_and_old_samples_are_not_trips() -> None:
    clock = Clock()
    pm = monitor(clock)
    pm.on_message("right", telemetry([6.0] * 9))
    pm.on_message("right", Event(EventCode.DISARMED, 0, 0, 0, (0, 1, 0, 0), "DISARM ALL: operator"))
    assert not pm.trips
    clock.t += 5.0  # beyond the look-back
    assert pm.summary()["right"].samples == 0
