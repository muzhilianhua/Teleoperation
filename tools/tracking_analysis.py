"""Servo tracking error from a Teach recording: how far each servo stays from
the goal it was sent, at rest and while moving.

Reads recordings/<id>/meta.json and <hand>.csv (with the `age` column, see
host/README.md). Uses only rows where a fresh servo read landed, and compares
each reading with the goal that was sent when it was taken (`age` ms before
the row), so the ~30 ms round-robin read period does not count as error.

Per joint:

- rest: the goal moved < 2 SCS steps (6.7 HLS) in the previous 1 s.
  pos - sent, split by the direction the goal last came from. A position loop
  without integral action stops short of the goal by load / stiffness, on the
  side it came from (friction) or always on one side (a spring or preload).
  `stiffness` is |load| / |error| in % per native step; on the HLS3606 the
  factory Kp 32 gives about 0.4 %/step (Kp / 8 per 0.1 %).
- moving: |goal velocity| above 50 SCS steps/s. A fit error = -lag x velocity
  + bias gives the servo's effective lag in ms.

Run it on one recording per setting (e.g. per `set <j> kp`) and compare.

Usage:
    python3 tools/tracking_analysis.py recordings/<id>
    python3 tools/tracking_analysis.py recordings/<id> --hand right --joints 6 8
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import statistics
import sys
from pathlib import Path

FOLLOW = 2  # JointState::Follow
HLS = 2  # feetech::ServoModel::Hls3606
HLS_SCALE = 3.333


def fit(xs: list[float], ys: list[float]) -> tuple[float, float]:
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return math.nan, my
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys, strict=True)) / sxx
    return a, my - a * mx


def rms(values: list[float]) -> float:
    return math.sqrt(statistics.fmean([v * v for v in values]))


def analyse_joint(rows: list[dict], t: list[float], k: int, name: str, hls: bool, rest_s: float) -> None:
    def col(field: str) -> list[float]:
        return [float(r[f"j{k}_{field}"]) for r in rows]

    state, goal, sent, pos, age, load = (col(f) for f in ("state", "goal", "sent", "pos", "age", "load"))
    contact = col("contact")
    scale = HLS_SCALE if hls else 1.0
    unit = "HLS" if hls else "SCS"

    def sent_at(tt: float) -> float:
        i = max(0, bisect.bisect_right(t, tt) - 1)
        return sent[i]

    # Goal velocity over 50 ms.
    gv = [0.0] * len(rows)
    for i in range(5, len(rows)):
        dt = t[i] - t[i - 5]
        gv[i] = (goal[i] - goal[i - 5]) / dt if dt > 0 else 0.0

    rest: list[tuple[float, int, float]] = []  # error, approach direction, load
    moving: list[tuple[float, float]] = []  # velocity, error
    rest_rows = round(rest_s * 100)
    for i in range(1, len(rows)):
        if not (age[i] < age[i - 1] and state[i] == FOLLOW) or contact[i]:
            continue
        taken = t[i] - age[i] / 1000.0
        error = pos[i] - sent_at(taken)
        j0 = i - rest_rows
        if j0 >= 0 and max(goal[j0 : i + 1]) - min(goal[j0 : i + 1]) < 2 * scale:
            direction = 0
            for j in range(j0, max(0, j0 - 1000), -1):
                if abs(goal[j0] - goal[j]) > 5 * scale:
                    direction = 1 if goal[j0] > goal[j] else -1
                    break
            rest.append((error, direction, load[i]))
        else:
            v = gv[max(0, bisect.bisect_right(t, taken) - 1)]
            if abs(v) > 50 * scale:
                moving.append((v, error))

    print(f"\nj{k} {name} ({'HLS3606' if hls else 'SCS0009'}, {unit} steps)")
    if rest:
        errors = [e for e, _, _ in rest]
        print(
            f"  rest     n {len(rest):4d}  |pos-sent| mean {statistics.fmean(abs(e) for e in errors):5.1f}"
            f"  max {max(abs(e) for e in errors):4.0f}"
        )
        for d, label in ((1, "from below"), (-1, "from above")):
            side = [e for e, dd, _ in rest if dd == d]
            if side:
                print(f"    {label}: n {len(side):4d}  pos-sent mean {statistics.fmean(side):+6.1f}")
        stiff = [abs(ld) / abs(e) for e, _, ld in rest if abs(e) >= 5 * scale]
        if stiff:
            print(f"    stiffness |load| / |error| {statistics.median(stiff):.2f} %/step (median)")
    else:
        print(f"  rest     none (glove never still for {rest_s:g} s)")
    if len(moving) > 10:
        vs = [v for v, _ in moving]
        es = [e for _, e in moving]
        a, b = fit(vs, es)
        print(
            f"  moving   n {len(moving):4d}  |v| median {statistics.median(abs(v) for v in vs):5.0f} steps/s"
            f"  rms {rms(es):5.1f}  max {max(abs(e) for e in es):4.0f}  lag {-a * 1000:5.1f} ms  bias {b:+.1f}"
        )
    else:
        print("  moving   too few samples")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recording", type=Path, help="recordings/<id> folder")
    parser.add_argument("--hand", choices=["right", "left"], help="needed when the recording has both hands")
    parser.add_argument("--joints", type=int, nargs="+", help="servo ids (default: the joints that followed)")
    parser.add_argument("--rest", type=float, default=1.0, help="seconds the goal must be still to count as rest")
    args = parser.parse_args(argv)

    meta_path = args.recording / "meta.json"
    if not meta_path.is_file():
        print(f"{args.recording}: no meta.json", file=sys.stderr)
        return 1
    meta = json.loads(meta_path.read_text())
    hands = list(meta["hands"])
    hand = args.hand or (hands[0] if len(hands) == 1 else None)
    if hand not in meta["hands"]:
        print(f"pick a hand with --hand (recording has {hands})", file=sys.stderr)
        return 1
    hmeta = meta["hands"][hand]
    table = {j["servo_id"]: j for j in hmeta["joints"]}
    with (args.recording / f"{hand}.csv").open(newline="") as f:
        rows = list(csv.DictReader(f))
    ids = args.joints or hmeta["active"]
    if rows and f"j{ids[0]}_age" not in rows[0]:
        print("recording predates the `age` column: record it again", file=sys.stderr)
        return 1
    t = [float(r["t"]) for r in rows]
    print(f"recording {meta['id']} ({meta['name']}), {hand} hand, {t[-1] - t[0]:.1f} s")
    for k in ids:
        info = table.get(k, {"name": f"j{k}", "model": 0})
        analyse_joint(rows, t, k, info["name"], info["model"] == HLS, args.rest)
    return 0


if __name__ == "__main__":
    sys.exit(main())
