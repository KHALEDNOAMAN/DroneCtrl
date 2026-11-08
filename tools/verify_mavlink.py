"""Decode the SIL .tlog files with pymavlink and check them against the CSV logs.

mavlink_telemetry.h is a hand-written encoder. The easy mistakes in MAVLink
are a wrong CRC_EXTRA seed, fields in XML order instead of wire order, and a
broken truncation rule, and every one of them produces frames that look fine
in a hex dump and are silently dropped by a ground station. pymavlink is
generated from the official message definitions, so if it decodes every frame
with a good checksum and the values match the CSV log of the same flight, the
encoder is right.

    python3 tools/verify_mavlink.py firmware/sil/logs

Exits non-zero on the first problem.
"""
import csv
import math
import sys
from pathlib import Path

from pymavlink import mavutil

QUADROTOR = 2
ARMED_FLAG = 128
ARMED_STATES = {2, 3}  # FlightState::ARMED, FlightState::FAILSAFE_LAND


def fail(msg):
    print(f"FAIL: {msg}")
    sys.exit(1)


def check(tlog: Path) -> int:
    rows = list(csv.DictReader(open(tlog.with_suffix(".csv"))))
    conn = mavutil.mavlink_connection(str(tlog), dialect="common", robust_parsing=False)

    attitudes, huds, beats, statuses = [], [], [], []
    last_seq = None
    while True:
        m = conn.recv_match()
        if m is None:
            break
        kind = m.get_type()
        if kind == "BAD_DATA":
            fail(f"{tlog.name}: undecodable or bad-checksum frame")
        seq = m.get_header().seq
        if last_seq is not None and seq != (last_seq + 1) % 256:
            fail(f"{tlog.name}: sequence jumped {last_seq} -> {seq}")
        last_seq = seq
        {"ATTITUDE": attitudes, "VFR_HUD": huds, "HEARTBEAT": beats,
         "SYS_STATUS": statuses}.get(kind, []).append(m)

    if len(attitudes) != len(rows):
        fail(f"{tlog.name}: {len(attitudes)} ATTITUDE messages for {len(rows)} CSV rows")
    if len(huds) != len(rows):
        fail(f"{tlog.name}: {len(huds)} VFR_HUD messages for {len(rows)} CSV rows")
    duration = float(rows[-1]["t"])
    if len(beats) < int(duration):
        fail(f"{tlog.name}: {len(beats)} heartbeats over {duration:.0f} s")

    for att, hud, row in zip(attitudes, huds, rows):
        t = float(row["t"])
        if abs(att.time_boot_ms - t * 1000) > 1:
            fail(f"{tlog.name}: time_boot_ms {att.time_boot_ms} vs t {t}")
        for got, col in ((att.roll, "roll_est"), (att.pitch, "pitch_est")):
            if abs(math.degrees(got) - float(row[col])) > 1e-3:
                fail(f"{tlog.name} t={t}: {col} {math.degrees(got):.4f} vs {row[col]}")
        if abs(hud.alt - float(row["z"])) > 1e-3:
            fail(f"{tlog.name} t={t}: alt {hud.alt} vs z {row['z']}")
        thr = round(min(max(float(row["throttle_cmd"]), 0.0), 1.0) * 100)
        if abs(hud.throttle - thr) > 1:
            fail(f"{tlog.name} t={t}: throttle {hud.throttle} vs {thr}")

    for hb in beats:
        if hb.type != QUADROTOR:
            fail(f"{tlog.name}: heartbeat type {hb.type}")
        if hb.mavlink_version != 3:
            fail(f"{tlog.name}: mavlink_version {hb.mavlink_version}")

    # The armed flag and custom mode must follow the state machine. Each
    # heartbeat is stamped with the time of the sample it was built from.
    t0 = attitudes[0]._timestamp - float(rows[0]["t"])
    times = [float(r["t"]) for r in rows]
    for hb in beats:
        t = hb._timestamp - t0
        row = rows[min(range(len(times)), key=lambda k: abs(times[k] - t))]
        if abs(float(row["t"]) - t) > 1e-3:
            fail(f"{tlog.name}: heartbeat at {t:.3f} s matches no CSV row")
        armed = bool(hb.base_mode & ARMED_FLAG)
        if armed != (int(row["state"]) in ARMED_STATES):
            fail(f"{tlog.name}: armed flag {armed} in state {row['state']}")
        if hb.custom_mode != int(row["state"]):
            fail(f"{tlog.name}: custom_mode {hb.custom_mode} vs state {row['state']}")

    for st in statuses:
        if not 0 < st.voltage_battery < 20000:
            fail(f"{tlog.name}: voltage_battery {st.voltage_battery} mV")

    print(f"  {tlog.name:28s} {len(attitudes) + len(huds) + len(beats) + len(statuses):6d} "
          f"frames, {len(beats)} heartbeats, all decoded and matched")
    return len(attitudes)


def main():
    folder = Path(sys.argv[1] if len(sys.argv) > 1 else "firmware/sil/logs")
    tlogs = sorted(folder.glob("*.tlog"))
    if not tlogs:
        fail(f"no .tlog files in {folder}")
    print(f"pymavlink check of {len(tlogs)} telemetry logs")
    for t in tlogs:
        check(t)
    print("all telemetry logs decode cleanly and match the CSV")


if __name__ == "__main__":
    main()
