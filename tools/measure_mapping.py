# /// script
# requires-python = ">=3.12"
# dependencies = ["matplotlib"]
# ///
"""Exoskeleton -> hand mapping consistency measurement.

Two steps:

1. ``extract``: pull every ``status`` snapshot out of a serial log
   (``pio device monitor | tee measure.log``) into a CSV, one row per
   snapshot and joint. Round / direction / pose are pre-filled assuming the
   measurement protocol (per round: A, 25, 50, 75, B while closing, then B,
   75, 50, 25, A while opening); fix them if you typed an extra ``status``.
   Fill in ``theta_h`` (human joint angle from the jig) and ``theta_r``
   (measured robot joint angle) by hand.

2. ``analyze``: per joint, report servo following error, linear fit of
   theta_r over theta_h, offset spread between rounds (re-donning), residual
   non-linearity and hysteresis, and say which mapping change the data
   points to. ``--plot DIR`` writes one PNG per joint (needs matplotlib).

Usage:
    uv run tools/measure_mapping.py extract measure.log --joints 1 6 9 -o poses.csv
    uv run tools/measure_mapping.py analyze poses.csv --plot plots/
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

# Matches console.cpp printStatus(); a timestamp prefix from the monitor's
# `time` filter (e.g. "14:03:22.123 > ") is tolerated because we search.
BLOCK_START = re.compile(r"armed \d+ joint\(s\)")
BLOCK_END = re.compile(r"global lead ")
JOINT_LINE = re.compile(
    r"j(?P<joint>\d+)\s+(?P<name>\S+)\s+(?P<state>[A-Z]+)\s+"
    r"exo\s+(?P<exo>-?\d+(?:\.\d+)?) deg\s+"
    r"target\s+(?P<target>-?\d+(?:\.\d+)?)\s+"
    r"goal\s+(?P<goal>-?\d+(?:\.\d+)?)\s+"
    r"sent\s+(?P<sent>-?\d+(?:\.\d+)?)\s+"
    r"grip\s+(?P<grip>\d+)(?P<contact>\*?)\s+"
    r"spd\s+\d+"
    r"(?:\s+pos\s+(?P<pos>-?\d+)\s+tq\s+(?P<tq>\d)\s+load\s+(?P<load>-?\d+(?:\.\d+)?)%)?"
    r"(?P<rest>.*)$"
)

PROTOCOL_POSES = ["A", "25", "50", "75", "B"]
CSV_FIELDS = [
    "snapshot",
    "joint",
    "name",
    "state",
    "exo_deg",
    "target",
    "goal",
    "sent",
    "grip",
    "contact",
    "pos",
    "load",
    "flag",
    "round",
    "direction",
    "pose",
    "theta_h",
    "theta_r",
]


# ---- extract --------------------------------------------------------------------


def parse_log(text: str) -> list[list[dict[str, str]]]:
    """Return the status snapshots in order, each a list of joint rows."""
    snapshots: list[list[dict[str, str]]] = []
    current: list[dict[str, str]] | None = None
    for line in text.splitlines():
        if BLOCK_START.search(line):
            current = []
            continue
        if current is None:
            continue
        if BLOCK_END.search(line):
            snapshots.append(current)
            current = None
            continue
        m = JOINT_LINE.search(line)
        if m:
            rest = m.group("rest")
            flag = "HOME!" if "HOME!" in rest else ("ENC!" if "ENC!" in rest else "")
            if "no recent reply" in rest:
                flag = (flag + " NOREPLY").strip()
            current.append(
                {
                    "joint": m.group("joint"),
                    "name": m.group("name"),
                    "state": m.group("state"),
                    "exo_deg": m.group("exo"),
                    "target": m.group("target"),
                    "goal": m.group("goal"),
                    "sent": m.group("sent"),
                    "grip": m.group("grip"),
                    "contact": "1" if m.group("contact") else "0",
                    "pos": m.group("pos") or "",
                    "load": m.group("load") or "",
                    "flag": flag,
                }
            )
    return snapshots


def protocol_labels(index: int) -> tuple[str, str, str]:
    """(round, direction, pose) for the n-th snapshot under the standard protocol."""
    per_round = 2 * len(PROTOCOL_POSES)
    round_no = index // per_round + 1
    k = index % per_round
    if k < len(PROTOCOL_POSES):
        return str(round_no), "close", PROTOCOL_POSES[k]
    return str(round_no), "open", PROTOCOL_POSES[per_round - 1 - k]


def cmd_extract(args: argparse.Namespace) -> int:
    text = Path(args.log).read_text(encoding="utf-8", errors="replace")
    snapshots = parse_log(text)
    if not snapshots:
        print("no `status` snapshots found in the log", file=sys.stderr)
        return 1
    wanted = {str(j) for j in args.joints} if args.joints else None
    rows = 0
    with open(args.output, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for i, snap in enumerate(snapshots):
            round_no, direction, pose = protocol_labels(i) if not args.no_protocol else ("", "", "")
            for row in snap:
                if wanted and row["joint"] not in wanted:
                    continue
                writer.writerow(
                    {
                        "snapshot": i + 1,
                        **row,
                        "round": round_no,
                        "direction": direction,
                        "pose": pose,
                        "theta_h": "",
                        "theta_r": "",
                    }
                )
                rows += 1
    per_round = 2 * len(PROTOCOL_POSES)
    print(f"{len(snapshots)} snapshot(s), {rows} row(s) -> {args.output}")
    if not args.no_protocol and len(snapshots) % per_round:
        print(
            f"note: {len(snapshots)} is not a multiple of {per_round}; check the round/direction/pose "
            "labels (an extra or missing `status` shifts every later row)"
        )
    print("next: fill in theta_h and theta_r (degrees), then run `analyze`")
    return 0


# ---- analyze --------------------------------------------------------------------


@dataclass
class Point:
    round: str
    direction: str
    pose: str
    theta_h: float
    theta_r: float
    follow: float | None  # pos - sent, servo steps
    contact: bool


def to_float(value: str) -> float | None:
    value = (value or "").strip()
    if not value:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def fit_common_slope(pts: list[Point]) -> tuple[float, dict[str, float]]:
    """y = a x + b_round: one slope, one intercept per round (pooled within-round fit).

    Separating the intercepts keeps a re-donning offset out of the slope and
    out of the residuals, so residuals show curvature and hysteresis only.
    """
    groups: dict[str, list[Point]] = defaultdict(list)
    for p in pts:
        groups[p.round].append(p)
    sxy = sxx = 0.0
    means: dict[str, tuple[float, float]] = {}
    for rnd, g in groups.items():
        mx = statistics.fmean(p.theta_h for p in g)
        my = statistics.fmean(p.theta_r for p in g)
        means[rnd] = (mx, my)
        sxx += sum((p.theta_h - mx) ** 2 for p in g)
        sxy += sum((p.theta_h - mx) * (p.theta_r - my) for p in g)
    a = sxy / sxx if sxx else math.nan
    return a, {rnd: my - a * mx for rnd, (mx, my) in sorted(means.items())}


def load_points(path: str, servo_deg: float | None) -> tuple[dict[str, list[Point]], dict[str, str], set[str]]:
    points: dict[str, list[Point]] = defaultdict(list)
    names: dict[str, str] = {}
    exo_fallback: set[str] = set()
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            joint = row["joint"]
            names[joint] = row.get("name", "")
            if not row.get("pose"):
                continue
            theta_h = to_float(row.get("theta_h", ""))
            if theta_h is None:
                # Without a jig angle we can still compare exoskeleton and robot.
                theta_h = to_float(row.get("exo_deg", ""))
                exo_fallback.add(joint)
            theta_r = to_float(row.get("theta_r", ""))
            pos = to_float(row.get("pos", ""))
            if theta_r is None and servo_deg is not None and pos is not None:
                theta_r = pos * servo_deg
            if theta_h is None or theta_r is None:
                continue
            sent = to_float(row.get("sent", ""))
            follow = pos - sent if pos is not None and sent is not None else None
            points[joint].append(
                Point(
                    row.get("round", ""),
                    row.get("direction", ""),
                    row["pose"],
                    theta_h,
                    theta_r,
                    follow,
                    row.get("contact") == "1",
                )
            )
    return points, names, exo_fallback


@dataclass
class JointResult:
    n: int
    slope: float
    offset: float  # mean of the per-round intercepts
    nonlinearity: float  # largest |pose-mean deviation from the A-B chord|
    residual_rms: float
    worst_pose: str
    pose_residuals: dict[str, float]
    point_residuals: list[float]  # aligned with the joint's points
    offset_spread: float
    round_offsets: dict[str, float]
    hysteresis_max: float
    hysteresis_mean: float
    follow_max: float | None
    contact_rows: int


def chord_residuals(pts: list[Point], slope: float, round_offsets: dict[str, float]) -> list[float]:
    """Deviation of each point from its round's straight line through poses A and B.

    The firmware maps linearly between the endpoints captured at A and B, so
    this is the error that mapping makes in between. A round without both
    endpoints falls back to the common-slope fit.
    """
    ends: dict[tuple[str, str], tuple[float, float]] = {}
    for rnd in {p.round for p in pts}:
        for pose in ("A", "B"):
            sel = [p for p in pts if p.round == rnd and p.pose == pose]
            if sel:  # both directions averaged: hysteresis does not tilt the chord
                ends[(rnd, pose)] = (
                    statistics.fmean(p.theta_h for p in sel),
                    statistics.fmean(p.theta_r for p in sel),
                )
    out = []
    for p in pts:
        a_end, b_end = ends.get((p.round, "A")), ends.get((p.round, "B"))
        if a_end and b_end and b_end[0] != a_end[0]:
            k = (b_end[1] - a_end[1]) / (b_end[0] - a_end[0])
            out.append(p.theta_r - (a_end[1] + k * (p.theta_h - a_end[0])))
        else:
            out.append(p.theta_r - (slope * p.theta_h + round_offsets[p.round]))
    return out


def analyze_joint(pts: list[Point]) -> JointResult:
    a, round_offsets = fit_common_slope(pts)
    residuals = chord_residuals(pts, a, round_offsets)
    # Averaging each pose over rounds and both directions cancels noise and
    # hysteresis, leaving the shape the two-point mapping cannot follow.
    by_pose: dict[str, list[float]] = defaultdict(list)
    pose_h: dict[str, list[float]] = defaultdict(list)
    for p, r in zip(pts, residuals):
        by_pose[p.pose].append(r)
        pose_h[p.pose].append(p.theta_h)
    pose_residuals = {
        pose: statistics.fmean(v) for pose, v in sorted(by_pose.items(), key=lambda kv: statistics.fmean(pose_h[kv[0]]))
    }
    worst = max(pose_residuals, key=lambda k: abs(pose_residuals[k]))
    spread = max(round_offsets.values()) - min(round_offsets.values()) if len(round_offsets) > 1 else 0.0

    # Hysteresis: same round and pose, closing vs opening.
    pairs: dict[tuple[str, str], dict[str, float]] = defaultdict(dict)
    for p in pts:
        pairs[(p.round, p.pose)][p.direction] = p.theta_r
    hyst = [abs(d["close"] - d["open"]) for d in pairs.values() if "close" in d and "open" in d]

    follows = [abs(p.follow) for p in pts if p.follow is not None]
    return JointResult(
        n=len(pts),
        slope=a,
        offset=statistics.fmean(round_offsets.values()),
        nonlinearity=abs(pose_residuals[worst]),
        residual_rms=math.sqrt(statistics.fmean(r * r for r in residuals)),
        worst_pose=f"pose {worst}",
        pose_residuals=pose_residuals,
        point_residuals=residuals,
        offset_spread=spread,
        round_offsets=round_offsets,
        hysteresis_max=max(hyst) if hyst else 0.0,
        hysteresis_mean=statistics.fmean(hyst) if hyst else 0.0,
        follow_max=max(follows) if follows else None,
        contact_rows=sum(p.contact for p in pts),
    )


def recommend(r: JointResult, args: argparse.Namespace) -> list[str]:
    notes = []
    if r.follow_max is not None and r.follow_max > args.follow_steps:
        notes.append(
            f"servo does not follow its goal (|pos - sent| up to {r.follow_max:.0f} steps): "
            "check limits / load / sweep first, the mapping numbers below are polluted"
        )
    if r.contact_rows:
        notes.append(f"{r.contact_rows} row(s) had the grip limiter in contact: those poses were not free motion")
    if r.offset_spread > args.offset_deg:
        notes.append(f"offset moves {r.offset_spread:.1f} deg between rounds -> scheme 1 (session zero)")
    if r.nonlinearity > args.nonlinear_deg:
        notes.append(
            f"non-linear: {r.nonlinearity:.1f} deg off the A-B line at {r.worst_pose} "
            "-> scheme 2 (3-point / piecewise mapping)"
        )
    if args.expected_gain and abs(r.slope - args.expected_gain) > args.gain_tolerance * args.expected_gain:
        notes.append(
            f"slope {r.slope:.2f} vs expected {args.expected_gain:.2f} -> scheme 3 (angle mapping) "
            "if equal angles are wanted"
        )
    if r.hysteresis_max > args.hysteresis_deg:
        notes.append(f"hysteresis up to {r.hysteresis_max:.1f} deg: backlash, not fixed by zero calibration")
    if not notes:
        notes.append("within thresholds: the two-point mapping is good enough for this joint")
    return notes


def plot_joint(joint: str, name: str, pts: list[Point], r: JointResult, out_dir: Path) -> Path | None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed: skipping plots (run with `uv run` to get it)", file=sys.stderr)
        return None
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 7), sharex=True, gridspec_kw={"height_ratios": [3, 2]})
    rounds = sorted({p.round for p in pts})
    for i, rnd in enumerate(rounds):
        for direction, marker in (("close", "o"), ("open", "s")):
            idx = [k for k, p in enumerate(pts) if p.round == rnd and p.direction == direction]
            if not idx:
                continue
            xs = [pts[k].theta_h for k in idx]
            ax1.scatter(
                xs, [pts[k].theta_r for k in idx], marker=marker, color=f"C{i}", label=f"round {rnd} {direction}"
            )
            ax2.scatter(xs, [r.point_residuals[k] for k in idx], marker=marker, color=f"C{i}")
    lo, hi = min(p.theta_h for p in pts), max(p.theta_h for p in pts)
    for i, (rnd, b) in enumerate(r.round_offsets.items()):
        ax1.plot(
            [lo, hi],
            [r.slope * lo + b, r.slope * hi + b],
            "--",
            color=f"C{i}",
            linewidth=1,
            label=f"round {rnd} fit {r.slope:.2f}x{b:+.1f}",
        )
    pose_x = {p.pose: [] for p in pts}
    for p in pts:
        pose_x[p.pose].append(p.theta_h)
    curve = sorted((statistics.fmean(pose_x[k]), v) for k, v in r.pose_residuals.items())
    ax2.plot([c[0] for c in curve], [c[1] for c in curve], "k-", linewidth=1.5, label="pose mean")
    ax2.legend(fontsize=8)
    ax1.set_ylabel("robot angle theta_r (deg)")
    ax1.set_title(f"joint {joint} {name}")
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.3)
    ax2.axhline(0, color="k", linewidth=0.8)
    ax2.set_xlabel("human / exo angle theta_h (deg)")
    ax2.set_ylabel("off the A-B line (deg)")
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"joint{joint}_{name}.png"
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def cmd_analyze(args: argparse.Namespace) -> int:
    points, names, exo_fallback = load_points(args.csv, args.theta_r_from_pos)
    if not points:
        print("no usable rows: fill in pose and theta_r (or pass --theta-r-from-pos)", file=sys.stderr)
        return 1
    print("| joint | n | slope | offset | non-linearity / rms | offset spread | hysteresis max | |pos-sent| max |")
    print("|---|---|---|---|---|---|---|---|")
    results = {}
    for joint in sorted(points, key=int):
        pts = points[joint]
        if len(pts) < 3:
            print(f"| {joint} | {len(pts)} | too few points | | | | | |")
            continue
        r = analyze_joint(pts)
        results[joint] = r
        follow = f"{r.follow_max:.0f}" if r.follow_max is not None else "-"
        print(
            f"| {joint} {names.get(joint, '')} | {r.n} | {r.slope:.3f} | {r.offset:+.1f} | "
            f"{r.nonlinearity:.1f} / {r.residual_rms:.1f} | {r.offset_spread:.1f} | "
            f"{r.hysteresis_max:.1f} | {follow} |"
        )
    print("\nunits: degrees, except |pos-sent| in servo steps (1 step ~ 0.293 deg)")
    for joint, r in results.items():
        print(f"\njoint {joint} {names.get(joint, '')}")
        if joint in exo_fallback:
            print("  - theta_h missing on some rows: used exo_deg (exoskeleton encoder angle) instead")
        if len(r.round_offsets) > 1:
            offsets = ", ".join(f"round {k}: {v:+.1f}" for k, v in r.round_offsets.items())
            print(f"  - offsets per round: {offsets}")
        shape = ", ".join(f"{k}: {v:+.1f}" for k, v in r.pose_residuals.items())
        print(f"  - pose-mean residual (curvature): {shape}")
        for note in recommend(r, args):
            print(f"  - {note}")
        if args.plot:
            path = plot_joint(joint, names.get(joint, ""), points[joint], r, Path(args.plot))
            if path:
                print(f"  - plot: {path}")
    return 0


# ---- main -----------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    ex = sub.add_parser("extract", help="status snapshots from a serial log -> CSV to fill in")
    ex.add_argument("log", help="serial monitor log (pio device monitor | tee measure.log)")
    ex.add_argument("--joints", type=int, nargs="*", help="servo ids to keep (default: all printed)")
    ex.add_argument("--no-protocol", action="store_true", help="leave round/direction/pose empty")
    ex.add_argument("-o", "--output", default="poses.csv")
    ex.set_defaults(func=cmd_extract)

    an = sub.add_parser("analyze", help="fit and diagnose a filled-in CSV")
    an.add_argument("csv")
    an.add_argument(
        "--theta-r-from-pos",
        type=float,
        metavar="DEG_PER_STEP",
        help="use pos x DEG_PER_STEP when theta_r is empty (0.293 for direct-drive SCS0009)",
    )
    an.add_argument("--plot", metavar="DIR", help="write one PNG per joint (needs matplotlib)")
    an.add_argument("--offset-deg", type=float, default=3.0, help="offset spread threshold (default 3)")
    an.add_argument("--nonlinear-deg", type=float, default=5.0, help="residual threshold (default 5)")
    an.add_argument("--hysteresis-deg", type=float, default=3.0, help="hysteresis threshold (default 3)")
    an.add_argument("--follow-steps", type=float, default=10.0, help="|pos - sent| threshold (default 10)")
    an.add_argument(
        "--expected-gain",
        type=float,
        default=0.0,
        help="expected theta_r/theta_h slope, e.g. 1.0 for equal angles (default: no check)",
    )
    an.add_argument("--gain-tolerance", type=float, default=0.1, help="relative slope tolerance (default 0.1)")
    an.set_defaults(func=cmd_analyze)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
