#!/usr/bin/env python3
"""Cross-check DroneCtrl's MAVLink against pymavlink, an independent implementation.

A codec tested only against itself proves it is self-consistent, not that a
ground station can read it. This checks both directions against pymavlink:

  encode  every .tlog the SIL harness wrote is read back with pymavlink's own
          tlog reader. Every frame must pass pymavlink's CRC, and the decoded
          telemetry must match the flight it came from (the CSV of the same
          scenario).

  decode  pymavlink encodes HIL_SENSOR, RC_CHANNELS_OVERRIDE and
          HIL_ACTUATOR_CONTROLS frames, including ones it truncates, and
          tools/mavlink_dump (our parser) must decode every field exactly.

Usage, from the repository root, after `make -C firmware/sil run`:

    pip install pymavlink
    python3 tools/mavlink_crosscheck.py firmware/sil/logs

Exit status is non-zero on any disagreement, so CI needs no extra parsing.
"""

import csv
import glob
import json
import math
import os
import random
import struct
import subprocess
import sys
import tempfile

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mav

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

failures = []


def fail(msg):
    failures.append(msg)
    print("  FAIL", msg)


def f32(x):
    """Round a Python float to binary32, as it is on the wire."""
    return struct.unpack("<f", struct.pack("<f", x))[0]


# ------------------------------------------------------------------ encode


def check_tlog(path, csv_path):
    name = os.path.basename(path)
    log = mavutil.mavlink_connection(path, dialect="common", robust_parsing=False)
    counts = {}
    bad = 0
    attitude = []
    states = []
    while True:
        m = log.recv_match()
        if m is None:
            break
        t = m.get_type()
        if t == "BAD_DATA":
            bad += 1
            continue
        counts[t] = counts.get(t, 0) + 1
        if t == "ATTITUDE":
            attitude.append((m.time_boot_ms, m.roll, m.pitch))
        elif t == "HEARTBEAT":
            states.append(m.custom_mode & 0xFF)

    if bad:
        fail(f"{name}: pymavlink rejected {bad} frames")
    for required in ("HEARTBEAT", "SYS_STATUS", "ATTITUDE", "NAMED_VALUE_FLOAT"):
        if counts.get(required, 0) == 0:
            fail(f"{name}: no {required} frames")

    # Attitude in the tlog must be the estimator's attitude in the CSV. The CSV
    # is written at 50 Hz and telemetry at 25 Hz, both on the 4 ms step grid,
    # so samples line up exactly on shared timestamps.
    est = {}
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            ms = int(round(float(row["t"]) * 1000)) + 4  # SIL logs t before now_ms += 4
            est[ms] = (float(row["roll_est"]), float(row["pitch_est"]), int(row["state"]))
    compared = 0
    worst = 0.0
    for ms, roll, pitch in attitude:
        if ms not in est:
            continue
        r_deg, p_deg, _ = est[ms]
        worst = max(worst, abs(math.degrees(roll) - r_deg), abs(math.degrees(pitch) - p_deg))
        compared += 1
    if compared < 10:
        fail(f"{name}: only {compared} ATTITUDE samples lined up with the CSV")
    elif worst > 1e-3:
        fail(f"{name}: ATTITUDE differs from the flight log by {worst:.5f} deg")

    final_state = list(est.values())[-1][2] if est else None
    if states and final_state is not None and states[-1] != final_state:
        fail(f"{name}: last HEARTBEAT state {states[-1]} but flight ended in {final_state}")

    total = sum(counts.values())
    print(f"  {name:<28} {total:5d} frames, 0 rejected, attitude matches log to {worst:.1e} deg"
          if not bad and compared >= 10 and worst <= 1e-3
          else f"  {name:<28} {total:5d} frames")


# ------------------------------------------------------------------ decode


def build_dump():
    out = os.path.join(tempfile.gettempdir(), "dronectrl_mavlink_dump")
    subprocess.check_call(
        ["g++", "-std=c++17", "-O1", "-Wall", "-Wextra",
         "-I", os.path.join(ROOT, "firmware", "include"),
         os.path.join(HERE, "mavlink_dump.cpp"), "-o", out])
    return out


def check_decode():
    dump = build_dump()

    class Sink:
        def __init__(self):
            self.buf = bytearray()

        def write(self, b):
            self.buf += b

    sink = Sink()
    m = mav.MAVLink(sink, srcSystem=255, srcComponent=190)
    rng = random.Random(1)
    sent = []
    for i in range(300):
        kind = i % 3
        if kind == 0:
            v = dict(time_usec=rng.randrange(0, 2**63), xacc=rng.uniform(-20, 20),
                     yacc=rng.uniform(-20, 20), zacc=rng.uniform(-20, 20),
                     xgyro=rng.uniform(-5, 5), ygyro=rng.uniform(-5, 5), zgyro=rng.uniform(-5, 5),
                     fields_updated=rng.randrange(0, 2**32), id=rng.randrange(0, 4))
            if i % 9 == 0:  # exercise truncation: trailing fields zero
                v.update(fields_updated=0, id=0)
            msg = m.hil_sensor_encode(v["time_usec"], v["xacc"], v["yacc"], v["zacc"],
                                      v["xgyro"], v["ygyro"], v["zgyro"], 0, 0, 0, 0, 0, 0, 0,
                                      v["fields_updated"], v["id"])
            sent.append((107, v))
        elif kind == 1:
            ch = [rng.randrange(1000, 2001) for _ in range(8)]
            if i % 7 == 0:
                ch[4:] = [0, 0, 0, 0]
            msg = m.rc_channels_override_encode(1, 1, *ch)
            sent.append((70, dict(chan=ch, target_system=1)))
        else:
            ctrl = [rng.uniform(0, 1) for _ in range(4)] + [0.0] * 12
            mode = rng.choice([0, 32, 160])
            msg = m.hil_actuator_controls_encode(rng.randrange(0, 2**63), ctrl, mode, 0)
            sent.append((93, dict(time_usec=msg.time_usec, controls=ctrl[:4], mode=mode)))
        m.send(msg)

    # Garbage between frames, as on a shared UART.
    stream = bytes(sink.buf)
    noisy = bytearray(b"boot text\r\n")
    noisy += stream

    proc = subprocess.run([dump], input=bytes(noisy), capture_output=True)
    lines = [json.loads(l) for l in proc.stdout.decode().splitlines()]
    if proc.returncode != 0:
        fail("decode: our parser reported CRC errors on pymavlink frames: " + proc.stderr.decode())
    if len(lines) != len(sent):
        fail(f"decode: pymavlink sent {len(sent)} frames, our parser returned {len(lines)}")
        return

    mismatches = 0
    for (msgid, want), got in zip(sent, lines):
        if got["msgid"] != msgid:
            mismatches += 1
            continue
        for k, v in want.items():
            g = got[k]
            if isinstance(v, list):
                same = all((f32(a) == f32(b)) if isinstance(a, float) else a == b for a, b in zip(v, g))
            elif isinstance(v, float):
                # %.9g round-trips a binary32 exactly, so compare as binary32.
                same = f32(v) == f32(g)
            else:
                same = v == g
            if not same:
                mismatches += 1
                if mismatches <= 5:
                    fail(f"decode: msg {msgid} field {k}: pymavlink {v}, ours {g}")
    if mismatches == 0:
        print(f"  decode: {len(sent)} pymavlink frames (HIL_SENSOR, RC_CHANNELS_OVERRIDE,"
              f" HIL_ACTUATOR_CONTROLS) decoded field for field")


def main():
    logs = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "firmware", "sil", "logs")
    tlogs = sorted(glob.glob(os.path.join(logs, "*.tlog")))
    if not tlogs:
        print(f"no .tlog files in {logs}; run `make -C firmware/sil run` first")
        return 1

    print("MAVLink cross-check against pymavlink\n")
    print("encode: SIL telemetry read back by pymavlink")
    for t in tlogs:
        check_tlog(t, t[:-5] + ".csv")
    print()
    check_decode()

    print()
    if failures:
        print(f"{len(failures)} disagreement(s)")
        return 1
    print("pymavlink and DroneCtrl agree in both directions")
    return 0


if __name__ == "__main__":
    sys.exit(main())
