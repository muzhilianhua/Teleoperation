"""A simulated hand behind the simulated device: settings, calibration, arming
and simple joint motion, speaking the console grammar.

It is not a model of the firmware's control law. It reproduces what the host
link exposes (states, events with their codes and args, refusals while armed,
the dump format) closely enough to develop and test the GUI without hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

from ..protocol.messages import (
    DeviceInfo,
    Event,
    EventCode,
    JointFlag,
    JointInfo,
    Severity,
    SlowEncoder,
    SlowJoint,
    SlowStatus,
    Telemetry,
    TelemetryJoint,
)

ENCODER_COUNTS = 16384
DEG_PER_COUNT = 360.0 / ENCODER_COUNTS
NO_ENCODER = 0xFF
SCS, HLS = 1, 2
STEP_SCALE = {SCS: 1.0, HLS: (300.0 / 1024.0) / (360.0 / 4096.0)}
MAX_SPEED = {SCS: 1000, HLS: 12492}
OFF, ALIGN, FOLLOW, HOLD, SWEEP = range(5)
STATE_NAMES = ["OFF", "ALIGN", "FOLLOW", "HOLD", "SWEEP"]
FAULT_NAMES = [
    "none",
    "comm lost",
    "servo error",
    "servo voltage error",
    "torque mismatch",
    "load hard limit",
    "load trip",
    "overheat",
    "supply voltage",
    "tracking error",
    "over-current",
]
EVENT_NAMES = [code.name.lower() for code in EventCode]
MODEL_NAMES = ["unknown", "SCS0009", "HLS3606"]
MIN_SPAN, MAX_SPAN = 182, 3 * ENCODER_COUNTS


@dataclass(frozen=True)
class JointDefaults:
    servo_id: int
    name: str
    model: int
    travel_min: int
    travel_max: int
    encoder: int


# A plausible hand: SCS0009 on 1/3/5, HLS3606 elsewhere, every travel defined.
DEFAULT_TABLE = [
    JointDefaults(1, "sim_j1", SCS, 250, 730, 3),
    JointDefaults(2, "sim_j2", HLS, 800, 1900, 2),
    JointDefaults(3, "sim_j3", SCS, 20, 460, 1),
    JointDefaults(4, "sim_j4", HLS, 1750, 3240, 0),
    JointDefaults(5, "sim_j5", SCS, 700, 1000, 6),
    JointDefaults(6, "sim_j6", HLS, 1790, 2910, 7),
    JointDefaults(7, "sim_j7", HLS, 2450, 3150, 8),
    JointDefaults(8, "sim_j8", HLS, 2610, 3410, 5),
    JointDefaults(9, "sim_j9", HLS, 2050, 2980, 4),
]


@dataclass
class Settings:
    enabled: bool = False
    encoder: int = NO_ENCODER
    exo_ref: int = 0
    exo_b: int = 0
    servo_a: int = 0
    servo_b: int = 0
    vmax: int = 400
    amax: int = 8000
    vsat: int = 50
    trk: int = 40
    grip: int = 60
    ilimit: int = 0
    itrip: int = 90


def default_settings(d: JointDefaults) -> Settings:
    scale = STEP_SCALE[d.model]
    return Settings(
        encoder=d.encoder,
        servo_a=d.travel_min,
        servo_b=d.travel_min,
        vmax=round(400 * scale),
        amax=round(8000 * scale),
        vsat=round(50 * scale),
        trk=round(40 * scale),
        grip=round(60 * scale),
    )


@dataclass
class Joint:
    d: JointDefaults
    s: Settings
    state: int = OFF
    fault: int = 0
    position: float = 0.0
    goal: float = 0.0
    target: float = 0.0
    torque: bool = False
    tracking: bool = False  # exo continuously tracked since `cal a`
    homed: bool = True
    limits: tuple[int, int] = (0, 0)
    sweep_until: float = 0.0
    sweep_result: tuple[int, int] | None = None

    @property
    def scale(self) -> float:
        return STEP_SCALE[self.d.model]

    def exo_calibrated(self) -> bool:
        return self.s.exo_b != 0

    def servo_calibrated(self) -> bool:
        return self.s.servo_a != self.s.servo_b

    def ready(self) -> bool:
        return self.s.enabled and self.s.encoder != NO_ENCODER and self.exo_calibrated() and self.servo_calibrated()

    def armed(self) -> bool:
        return self.state != OFF


@dataclass
class Result:
    """Outcome of one command line."""

    parse_error: str | None = None
    ok: bool = True
    events: list[Event] = field(default_factory=list)
    logs: list[str] = field(default_factory=list)
    config: str | None = None


DISARMED_VERBS = {"cal", "save", "load", "defaults", "ping", "limits", "sweep"}
DISARMED_SET_KEYS = {"on", "off", "enc", "exo", "exospan", "servo"}
JOINT_NUMBER_KEYS = {"vmax", "amax", "vsat", "trk", "grip", "ilimit", "itrip"}
GLOBAL_KEYS = {"lead": "lead_ms", "mincut": "min_cutoff_hz", "beta": "beta", "jump": "jump_deg", "gripload": "gripload"}


class HandModel:
    def __init__(self, table: list[JointDefaults] | None = None) -> None:
        self.table = table or DEFAULT_TABLE
        self.joints = [Joint(d, default_settings(d), position=(d.travel_min + d.travel_max) / 2) for d in self.table]
        for j in self.joints:
            j.goal = j.target = j.position
        self.globals = {"lead_ms": 0.0, "min_cutoff_hz": 1.0, "beta": 0.01, "jump_deg": 25.0, "gripload": 60.0}
        self.load_trip = False
        self.monitor = False
        self.hand_fault = 0
        self.nvs: tuple[list[Settings], dict[str, float], bool] | None = None
        self.time = 0.0
        self.cycle = 0
        self.overruns = 0
        self._result: Result | None = None

    # ---- device info / telemetry ------------------------------------------------

    def device_info(self) -> DeviceInfo:
        joints = [
            JointInfo(
                d.servo_id,
                d.model,
                d.travel_min,
                d.travel_max,
                round(400 * STEP_SCALE[d.model]),
                round(50 * STEP_SCALE[d.model]),
                round(8000 * STEP_SCALE[d.model]),
                MAX_SPEED[d.model],
                d.encoder,
                0,
                STEP_SCALE[d.model],
                (300 / 1024) / STEP_SCALE[d.model],
                d.name,
            )
            for d in self.table
        ]
        return DeviceInfo(
            1,
            "sim",
            100,
            DEG_PER_COUNT,
            9,
            7,
            13306,
            50,
            100,
            90,
            MIN_SPAN,
            MAX_SPAN,
            joints,
            FAULT_NAMES,
            STATE_NAMES,
            EVENT_NAMES,
            MODEL_NAMES,
        )

    def encoder_raw(self, index: int) -> int:
        # Each encoder follows its own slow glove motion, about +-40 deg.
        angle = 4096 + 1500 * index + 1800 * math.sin(0.6 * self.time + index)
        return int(angle) % ENCODER_COUNTS

    def exo_counts(self, j: Joint) -> float:
        if j.s.encoder == NO_ENCODER:
            return 0.0
        diff = (self.encoder_raw(j.s.encoder) - j.s.exo_ref) % ENCODER_COUNTS
        return float(diff - ENCODER_COUNTS if diff > ENCODER_COUNTS // 2 else diff)

    def telemetry(self, t_us: int) -> Telemetry:
        joints = []
        for j in self.joints:
            flags = JointFlag.MODEL_OK | JointFlag.SERVO_OK | JointFlag.SERVO_FRESH | JointFlag.HOMED
            flags |= JointFlag.SINGLE_TURN
            if j.s.enabled:
                flags |= JointFlag.ENABLED
            if j.ready():
                flags |= JointFlag.READY
            if j.tracking and not j.exo_calibrated():
                flags |= JointFlag.REF_CAPTURED
            if j.s.encoder != NO_ENCODER:
                flags |= JointFlag.ENC_FRESH
            if j.torque:
                flags |= JointFlag.TORQUE_ON | JointFlag.GOAL_SENT
            if abs(j.goal - j.position) > 2:
                flags |= JointFlag.MOVING
            if j.d.model == HLS:
                flags |= JointFlag.HAS_CURRENT
            exo = self.exo_counts(j)
            load = 8.0 * (j.goal - j.position) / max(j.scale, 1.0) if j.torque else 0.0
            joints.append(
                TelemetryJoint(
                    state=j.state,
                    fault=j.fault,
                    flags=int(flags),
                    enc_raw=self.encoder_raw(j.s.encoder) if j.s.encoder != NO_ENCODER else 0,
                    exo_counts=exo,
                    exo_filtered_counts=exo,
                    target=j.target,
                    goal=j.goal,
                    sent_goal=j.goal,
                    reference=j.position,
                    speed_command=min(int(j.s.vmax * 1.3), 65535),
                    grip_lead=j.s.grip,
                    position=round(j.position),
                    speed=0,
                    load_deci_pct=max(-1000, min(1000, round(load * 10))),
                    current_ma=round(abs(load) * 30) if j.d.model == HLS else 0,
                    voltage_deci_v=60,
                    temperature_c=33 + j.d.servo_id % 4,
                    read_age_ms=10,
                )
            )
        armed = sum(j.armed() for j in self.joints)
        hand_flags = 1 | (2 if self.monitor else 0) | (4 if any(j.state == SWEEP for j in self.joints) else 0)
        return Telemetry(
            self.cycle, t_us & 0xFFFFFFFF, int(self.time * 1000), armed, self.hand_fault, hand_flags, 900, joints
        )

    def slow_status(self) -> SlowStatus:
        g = self.globals
        joints = [
            SlowJoint(
                int(j.s.enabled),
                j.s.encoder,
                j.s.exo_ref,
                j.s.exo_b,
                j.s.servo_a,
                j.s.servo_b,
                j.s.vmax,
                j.s.amax,
                j.s.vsat,
                j.s.trk,
                j.s.grip,
                j.s.ilimit,
                j.s.itrip,
                j.d.model,
                (min(j.s.ilimit, 5200) if j.s.ilimit else 5200) if j.d.model == HLS and j.armed() else 0,
                0,
            )
            for j in self.joints
        ]
        encoders = [SlowEncoder(0x40 + 4 * i, 0, 1, self.encoder_raw(i), 80, 1, 0) for i in range(9)]
        return SlowStatus(
            lead_ms=g["lead_ms"],
            min_cutoff_hz=g["min_cutoff_hz"],
            beta=g["beta"],
            jump_deg=g["jump_deg"],
            grip_load_pct=g["gripload"],
            load_trip=int(self.load_trip),
            cycles=self.cycle,
            overruns=self.overruns,
            last_period_us=10000,
            max_jitter_us=40,
            p99_jitter_us=20,
            last_exec_us=900,
            max_exec_us=1900,
            average_exec_us=880,
            bus_sent=self.cycle * 4,
            bus_received=self.cycle * 4,
            bus_last_reply_us=180,
            bus_max_reply_us=900,
            encoder_average_us=1500,
            encoder_max_us=1700,
            encoder_max_skew_us=40,
            monitor=int(self.monitor),
            monitor_ok_mask=0x1FF if self.monitor else 0,
            monitor_position=[round(j.position) for j in self.joints],
            joints=joints,
            encoders=encoders,
        )

    # ---- motion -------------------------------------------------------------------

    def tick(self, dt: float) -> list[Event]:
        """Advances one control cycle; returns unsolicited events."""
        self.time += dt
        self.cycle += 1
        events: list[Event] = []
        for j in self.joints:
            if j.state in (ALIGN, FOLLOW):
                exo = self.exo_counts(j)
                t = min(1.0, max(0.0, exo / j.s.exo_b)) if j.s.exo_b else 0.0
                j.target = j.s.servo_a + t * (j.s.servo_b - j.s.servo_a)
                speed = (50 * j.scale) if j.state == ALIGN else j.s.vmax
                step = max(-speed * dt, min(speed * dt, j.target - j.goal))
                j.goal += step
                if j.state == ALIGN and abs(j.goal - j.target) <= 3 * j.scale:
                    j.state = FOLLOW
                    events.append(
                        self._event(
                            EventCode.JOINT_STATE, j.d.servo_id, (FOLLOW,), f"joint {j.d.servo_id} aligned -> FOLLOW"
                        )
                    )
            elif j.state == SWEEP:
                j.goal = j.d.travel_min + (j.d.travel_max - j.d.travel_min) * (0.5 + 0.5 * math.sin(4 * self.time))
                if self.time >= j.sweep_until:
                    events += self._finish_sweep(j)
            if j.torque:
                j.position += (j.goal - j.position) * min(1.0, dt / 0.04)
        return events

    def _finish_sweep(self, j: Joint) -> list[Event]:
        j.state = OFF
        j.torque = False
        vmax, vsat = round(520 * j.scale), round(250 * j.scale)
        j.sweep_result = (vmax, vsat)
        i = j.d.servo_id
        return [
            self._event(
                EventCode.GENERIC,
                i,
                (),
                f"sweep j{i} done: saturation + {vmax / 1.2:.0f} steps/s, - {vsat / 0.9:.0f} steps/s",
            ),
            self._event(
                EventCode.SWEEP_DONE,
                i,
                (vmax, vsat, 1, 0),
                f"sweep j{i} suggests vmax {vmax} vsat {vsat} (now {j.s.vmax} / {j.s.vsat}): "
                f"`sweep {i} apply`, then `save`",
            ),
        ]

    # ---- commands ---------------------------------------------------------------------

    def _event(
        self, code: EventCode, joint: int, args: tuple[int, ...], text: str, severity: Severity = Severity.INFO
    ) -> Event:
        padded = (tuple(args) + (0, 0, 0, 0))[:4]
        return Event(code, severity, joint, 0, padded, text)  # type: ignore[arg-type]

    def _emit(
        self, code: EventCode, joint: int, args: tuple[int, ...], text: str, severity: Severity = Severity.INFO
    ) -> None:
        assert self._result is not None
        self._result.events.append(self._event(code, joint, args, text, severity))

    def _refuse(self, text: str) -> None:
        assert self._result is not None
        self._result.ok = False
        self._emit(EventCode.REFUSED, 0, (), text, Severity.ERROR)

    def joint(self, servo_id: int) -> Joint | None:
        return next((j for j in self.joints if j.d.servo_id == servo_id), None)

    def armed_count(self) -> int:
        return sum(j.armed() for j in self.joints)

    def handle(self, line: str) -> Result:
        self._result = result = Result()
        tokens = line.strip().lower().split()
        if not tokens or tokens[0].startswith("#"):
            return result
        verb, args = tokens[0], tokens[1:]
        handler = getattr(self, f"_cmd_{verb}", None)
        if handler is None:
            result.parse_error = "unknown command (type `help`)"
            return result
        needs_disarm = verb in DISARMED_VERBS or (verb == "monitor" and args == ["on"])
        needs_disarm |= verb == "set" and len(args) >= 2 and args[1] in DISARMED_SET_KEYS
        needs_disarm &= not (verb == "sweep" and len(args) == 2 and args[1] == "apply")
        try:
            if needs_disarm and self.armed_count() > 0:
                self._refuse("refused while joints are armed: disarm first")
                return result
            handler(args)
        except _ParseError as exc:
            result.parse_error = str(exc)
        return result

    # Console-only verbs

    def _cmd_help(self, args: list[str]) -> None:
        assert self._result is not None
        self._result.logs += ["joints are addressed by servo id (1-9)", "  (simulated device)"]

    def _cmd_status(self, args: list[str]) -> None:
        assert self._result is not None
        self._result.logs.append(f"armed {self.armed_count()} joint(s)  monitor {int(self.monitor)}  (simulated)")

    _cmd_stats = _cmd_enc = _cmd_joints = _cmd_status

    def _cmd_plot(self, args: list[str]) -> None:
        pass

    def _cmd_dump(self, args: list[str]) -> None:
        assert self._result is not None
        self._result.config = self.dump()

    def dump(self) -> str:
        out = ["defaults"]
        for j in self.joints:
            s, i = j.s, j.d.servo_id
            out.append(f"# {i} {j.d.name}")
            out.append(f"set {i} enc {'none' if s.encoder == NO_ENCODER else s.encoder}")
            if j.exo_calibrated():
                out.append(f"set {i} exospan {s.exo_ref} {s.exo_b}")
            if j.servo_calibrated():
                out.append(f"set {i} servo {s.servo_a} {s.servo_b}")
            out += [
                f"set {i} vmax {s.vmax}",
                f"set {i} amax {s.amax}",
                f"set {i} vsat {s.vsat}",
                f"set {i} trk {s.trk}",
                f"set {i} grip {s.grip}",
            ]
            if j.d.model == HLS:
                out.append(f"set {i} ilimit {s.ilimit}")
            out.append(f"set {i} {'on' if s.enabled else 'off'}")
        g = self.globals
        out += [
            f"set lead {g['lead_ms']:g}",
            f"set mincut {g['min_cutoff_hz']:g}",
            f"set beta {g['beta']:g}",
            f"set jump {g['jump_deg']:g}",
            f"set gripload {g['gripload']:g}",
            f"set loadtrip {'on' if self.load_trip else 'off'}",
            "# end of dump: run `save` to store it",
        ]
        return "\n".join(out) + "\n"

    # Controller verbs

    def _joint_arg(self, args: list[str], index: int = 0) -> Joint | None:
        if len(args) <= index or not args[index].isdigit():
            raise _ParseError("usage: <verb> <joint>")
        j = self.joint(int(args[index]))
        if j is None:
            self._refuse(f"no joint with servo id {args[index]}")
        return j

    def _cmd_arm(self, args: list[str]) -> None:
        if any(j.state == SWEEP for j in self.joints):
            self._refuse("refused while a sweep is running (`disarm` or `x` stops it)")
            return
        targets = [self._joint_arg(args)] if args else [j for j in self.joints if j.s.enabled and not j.armed()]
        if args and targets[0] is None:
            return
        if not targets:
            if self.armed_count():
                self._emit(EventCode.GENERIC, 0, (), "every enabled joint is already armed")
            else:
                self._refuse("arm refused: no joint enabled (set <j> on)")
            return
        for j in targets:
            assert j is not None
            if j.armed():
                self._refuse(f"joint {j.d.servo_id} is already armed")
                return
            if not j.ready():
                self._refuse(f"arm refused: joint {j.d.servo_id} not ready")
                return
        for j in targets:
            assert j is not None
            j.state, j.torque, j.goal, j.fault = ALIGN, True, j.position, 0
        if args:
            i = targets[0].d.servo_id  # type: ignore[union-attr]
            self._emit(EventCode.ARMED, i, (1,), f"joint {i} armed, holding -> ALIGN")
        else:
            self._emit(
                EventCode.ARMED,
                0,
                (len(targets),),
                f"armed {len(targets)} joint(s), holding -> ALIGN (keep the glove still)",
            )
        self.monitor = False

    def _cmd_disarm(self, args: list[str]) -> None:
        if args:
            j = self._joint_arg(args)
            if j is None:
                return
            if not j.armed():
                self._refuse(f"joint {j.d.servo_id} is not armed")
                return
            j.state, j.torque = OFF, False
            self._emit(
                EventCode.DISARMED,
                j.d.servo_id,
                (0, 1),
                f"joint {j.d.servo_id} ({j.d.name}) DISARM: operator; torque off confirmed; other joints unaffected",
            )
            return
        for j in self.joints:
            j.state, j.torque = OFF, False
        self._emit(EventCode.DISARMED, 0, (0, 1), "DISARM ALL: operator; torque off confirmed")

    def estop(self, reason: str = "E-stop (console)") -> Event:
        for j in self.joints:
            j.state, j.torque = OFF, False
        return self._event(EventCode.TORQUE_OFF, 0, (1,), f"{reason}: torque off on all servos", Severity.WARNING)

    def _cmd_x(self, args: list[str]) -> None:
        assert self._result is not None
        self._result.events.append(self.estop())

    def _cmd_ping(self, args: list[str]) -> None:
        for j in self.joints:
            self._emit(
                EventCode.PING_RESULT,
                j.d.servo_id,
                (j.d.model, 3, 43, 1),
                f"servo {j.d.servo_id} ({j.d.name}): {MODEL_NAMES[j.d.model]}, firmware 3.43",
            )

    def _cmd_stats(self, args: list[str]) -> None:  # noqa: F811  (`stats reset` goes to the controller)
        assert self._result is not None
        if args == ["reset"]:
            self.overruns = 0
            self._emit(EventCode.GENERIC, 0, (), "statistics cleared")
        else:
            self._result.logs.append(f"loop {self.cycle} cycles (simulated)")

    def _cmd_monitor(self, args: list[str]) -> None:
        if args not in (["on"], ["off"]):
            raise _ParseError("usage: monitor on|off")
        self.monitor = args == ["on"]
        self._emit(EventCode.MONITOR, 0, (int(self.monitor),), f"monitor {args[0]}")

    def _cmd_cal(self, args: list[str]) -> None:
        j = self._joint_arg(args)
        if j is None:
            return
        rest = args[1:]
        if rest in (["hand", "a"], ["hand", "b"]):
            b = rest[1] == "b"
            pos = round(j.position)
            if b:
                j.s.servo_b = pos
            else:
                j.s.servo_a = pos
            self._emit(
                EventCode.CAL_CAPTURED,
                j.d.servo_id,
                (3 if b else 2, pos),
                f"joint {j.d.servo_id}: servo {'B' if b else 'A'} = {pos}",
            )
            return
        if rest not in (["a"], ["b"]):
            raise _ParseError("usage: cal <joint> a|b  or  cal <joint> hand a|b")
        if j.s.encoder == NO_ENCODER:
            self._refuse(f"joint {j.d.servo_id}: assign an encoder first (set {j.d.servo_id} enc <n>)")
            return
        if rest == ["a"]:
            raw = self.encoder_raw(j.s.encoder)
            j.s.exo_ref, j.s.exo_b, j.tracking = raw, 0, True
            self._emit(
                EventCode.CAL_CAPTURED,
                j.d.servo_id,
                (0, raw),
                f"joint {j.d.servo_id}: exo A captured (raw {raw}). Move to the other end, through as many "
                f"turns as it takes, and run `cal {j.d.servo_id} b`.",
            )
            return
        if not j.tracking:
            self._refuse(
                f"joint {j.d.servo_id}: tracking was interrupted since `cal {j.d.servo_id} a` "
                f"(or it was not run): redo `cal {j.d.servo_id} a`"
            )
            return
        span = round(self.exo_counts(j))
        if not MIN_SPAN <= abs(span) <= MAX_SPAN:
            self._refuse(f"joint {j.d.servo_id}: span {span * DEG_PER_COUNT:.1f} deg rejected (4 deg .. 3 turns)")
            return
        j.s.exo_b = span
        self._emit(
            EventCode.CAL_CAPTURED,
            j.d.servo_id,
            (1, span),
            f"joint {j.d.servo_id}: exo B captured, span {span * DEG_PER_COUNT:.1f} deg",
        )

    def _cmd_home(self, args: list[str]) -> None:
        j = self._joint_arg(args)
        if j is None:
            return
        if j.armed():
            self._refuse(f"joint {j.d.servo_id} is armed: `disarm {j.d.servo_id}` before homing")
            return
        j.homed = True
        self._emit(EventCode.HOMED, j.d.servo_id, (0,), f"joint {j.d.servo_id} homed: 0.0 deg from A")

    def _cmd_set(self, args: list[str]) -> None:
        if len(args) == 2 and args[0] == "loadtrip" and args[1] in ("on", "off"):
            self.load_trip = args[1] == "on"
            self._global_changed()
            return
        if len(args) == 2 and args[0] in GLOBAL_KEYS:
            try:
                self.globals[GLOBAL_KEYS[args[0]]] = float(args[1])
            except ValueError as exc:
                raise _ParseError("usage: set <lead|mincut|beta|jump|gripload> <value>") from exc
            self._global_changed()
            return
        if len(args) < 2 or not args[0].isdigit():
            raise _ParseError("usage: set <joint> <key> <value...> (joint = servo id)")
        j = self._joint_arg(args)
        if j is None:
            return
        key, values = args[1], args[2:]
        s = replace(j.s)
        try:
            if key in ("on", "off"):
                s.enabled = key == "on"
            elif key == "enc":
                s.encoder = NO_ENCODER if values == ["none"] else int(values[0])
            elif key == "exospan":
                s.exo_ref, s.exo_b = int(values[0]), int(values[1])
            elif key == "servo":
                s.servo_a, s.servo_b = int(values[0]), int(values[1])
            elif key in JOINT_NUMBER_KEYS:
                setattr(s, key, int(values[0]))
            else:
                raise _ParseError("unknown key")
        except (IndexError, ValueError) as exc:
            raise _ParseError(f"usage: set <joint> {key} <value>") from exc
        error = self._validate(j, s)
        if error:
            self._refuse(f"set rejected: {error}")
            return
        j.s = s
        self._emit(
            EventCode.SETTING_CHANGED,
            j.d.servo_id,
            (),
            f"joint {j.d.servo_id}: {'on' if s.enabled else 'off'} enc {s.encoder} vmax {s.vmax}",
        )

    def _validate(self, j: Joint, s: Settings) -> str | None:
        if s.enabled and j.d.travel_min == j.d.travel_max:
            return "travel not defined in config.h: joint cannot be enabled"
        if s.encoder != NO_ENCODER and s.encoder >= 9:
            return "encoder index out of range"
        if s.servo_a != s.servo_b and not all(j.d.travel_min <= v <= j.d.travel_max for v in (s.servo_a, s.servo_b)):
            return "servo endpoint outside travel"
        if not 1 <= s.vmax <= MAX_SPEED[j.d.model] or not 1 <= s.vsat <= MAX_SPEED[j.d.model]:
            return "vmax outside 1..max speed of the servo model"
        if s.ilimit and (j.d.model != HLS or not 7 <= s.ilimit <= 13306):
            return "ilimit applies to HLS3606 joints only" if j.d.model != HLS else "ilimit outside 7..13306 mA"
        if s.itrip != 90 and (j.d.model != HLS or (s.itrip != 0 and not 50 <= s.itrip <= 100)):
            return "itrip must be 0 (off) or 50..100 %"
        return None

    def _global_changed(self) -> None:
        g = self.globals
        self._emit(
            EventCode.SETTING_CHANGED,
            0,
            (),
            f"global: lead {g['lead_ms']:.0f} ms, mincut {g['min_cutoff_hz']:.3f} Hz, beta {g['beta']:.4f}, "
            f"jump {g['jump_deg']:.0f} deg, gripload {g['gripload']:.0f} %, "
            f"loadtrip {'on' if self.load_trip else 'off'}",
        )

    def _cmd_save(self, args: list[str]) -> None:
        self.nvs = ([replace(j.s) for j in self.joints], dict(self.globals), self.load_trip)
        self._emit(EventCode.CONFIG_SAVED, 0, (1,), "configuration saved to NVS")

    def _cmd_load(self, args: list[str]) -> None:
        if self.nvs is None:
            self._result.ok = False  # type: ignore[union-attr]
            self._emit(EventCode.CONFIG_LOADED, 0, (-1,), "no stored configuration", Severity.WARNING)
            return
        settings, globals_, load_trip = self.nvs
        for j, s in zip(self.joints, settings, strict=True):
            j.s = replace(s)
        self.globals, self.load_trip = dict(globals_), load_trip
        self._emit(EventCode.CONFIG_LOADED, 0, (0,), "configuration loaded from NVS")

    def _cmd_defaults(self, args: list[str]) -> None:
        for j in self.joints:
            j.s = default_settings(j.d)
            j.tracking = False
        self.globals = {"lead_ms": 0.0, "min_cutoff_hz": 1.0, "beta": 0.01, "jump_deg": 25.0, "gripload": 60.0}
        self.load_trip = False
        self._emit(EventCode.DEFAULTS, 0, (), "defaults loaded (not saved; run `save` to keep them)")

    def _cmd_limits(self, args: list[str]) -> None:
        if args == ["check"]:
            mismatches = 0
            for j in self.joints:
                lo, hi = j.limits
                match = (lo, hi) == (j.d.travel_min, j.d.travel_max)
                mismatches += not match
                self._emit(
                    EventCode.LIMITS_ROW,
                    j.d.servo_id,
                    (lo, hi, int(match)),
                    f"limits j{j.d.servo_id} {j.d.name:<13} servo {lo:4}..{hi:<4}  travel "
                    f"{j.d.travel_min:4}..{j.d.travel_max:<4}  {'ok' if match else 'MISMATCH'}",
                )
            self._emit(
                EventCode.GENERIC,
                0,
                (),
                f"{mismatches} mismatch(es): `limits write all` syncs them"
                if mismatches
                else "all defined joints match",
            )
            return
        if len(args) == 2 and args[0] == "write":
            ids = [j.d.servo_id for j in self.joints] if args[1] == "all" else [int(args[1])]
            for i in ids:
                j = self.joint(i)
                if j is None:
                    self._refuse(f"no joint with servo id {i}")
                    return
                old = j.limits
                j.limits = (j.d.travel_min, j.d.travel_max)
                self._emit(
                    EventCode.GENERIC,
                    i,
                    (),
                    f"limits j{i}: {old[0]}..{old[1]} -> {j.limits[0]}..{j.limits[1]} written and verified",
                )
            return
        raise _ParseError("usage: limits check | limits write <joint|all>")

    def _cmd_sweep(self, args: list[str]) -> None:
        j = self._joint_arg(args)
        if j is None:
            return
        if args[1:] == ["apply"]:
            if j.sweep_result is None:
                self._refuse(f"joint {j.d.servo_id} has no sweep result yet: run `sweep {j.d.servo_id}` first")
                return
            j.s.vmax, j.s.vsat = j.sweep_result
            self._emit(
                EventCode.SETTING_CHANGED, j.d.servo_id, (), f"joint {j.d.servo_id}: vmax {j.s.vmax} vsat {j.s.vsat}"
            )
            return
        j.state, j.torque, j.sweep_until = SWEEP, True, self.time + 2.0
        i = j.d.servo_id
        self._emit(
            EventCode.GENERIC,
            i,
            (),
            f"sweep j{i} ({j.d.name}, {MODEL_NAMES[j.d.model]}): range "
            f"{j.d.travel_min}..{j.d.travel_max}, both directions",
        )
        for level, speed in enumerate((100, 300, 600, 1000)):
            cmd = round(speed * j.scale)
            self._emit(
                EventCode.SWEEP_RUN,
                i,
                (cmd, cmd * 10 * 9 // 10, 1, 300 + 50 * level),
                f"sweep j{i} {cmd:5} +: {cmd * 0.9:5.0f} steps/s arrived  , max load {30 + 5 * level:.0f} %",
            )


class _ParseError(Exception):
    pass
