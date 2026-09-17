#!/usr/bin/env python3
"""
Turn the SIL flight logs into the figures used in the README.

Reading a flight log is most of what post-flight analysis is, so the plots are
deliberately the ones an engineer would actually draw after a test: the
estimate against truth, the estimator's own health, what the failsafe did
second by second, and the actuator traces that decide whether a loop is stable.

Usage:
    python3 tools/plot_flight_logs.py [--logs firmware/sil/logs] [--out assets]

Only numpy and matplotlib are needed. Run firmware/sil first to produce the
logs; this script never invents data and exits non-zero if a log is missing.
"""

import argparse
import csv
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# Categorical palette, fixed order, never cycled. Checked for colour-vision
# separation rather than chosen by eye: the obvious red and green pairing fails
# for deuteranopia, where the two sit 5 units apart in perceptual distance and
# are effectively the same colour.
BLUE, AMBER, MAGENTA, TEAL = "#2563eb", "#d97706", "#be185d", "#0d9488"

# Status colours are reserved and never reused as a series colour.
STATUS = {
    "DISARMED": "#94a3b8",
    "ARMING": "#a8a29e",
    "ARMED": "#34d399",
    "FAILSAFE_LAND": "#fbbf24",
    "FAILSAFE_CUT": "#f87171",
}
STATE_NAMES = ["DISARMED", "ARMING", "ARMED", "FAILSAFE_LAND", "FAILSAFE_CUT"]
FAULT_NAMES = ["NONE", "SIGNAL_LOST", "LOW_BATTERY", "EXCESSIVE_TILT", "ESTIMATOR_DIVERGED"]

SURFACE = "#fcfcfb"
INK = "#1c1917"
INK_MUTED = "#78716c"
GRID = "#e7e5e4"


def style():
    """One recessive, print-safe style. Grid and axes stay out of the way."""
    plt.rcParams.update(
        {
            "figure.facecolor": SURFACE,
            "axes.facecolor": SURFACE,
            "savefig.facecolor": SURFACE,
            "axes.edgecolor": GRID,
            "axes.labelcolor": INK_MUTED,
            "axes.titlecolor": INK,
            "axes.titlesize": 11,
            "axes.titleweight": "semibold",
            "axes.labelsize": 9,
            "axes.grid": True,
            "grid.color": GRID,
            "grid.linewidth": 0.8,
            "xtick.color": INK_MUTED,
            "ytick.color": INK_MUTED,
            "xtick.labelsize": 8.5,
            "ytick.labelsize": 8.5,
            "legend.fontsize": 8.5,
            "legend.frameon": False,
            "font.size": 9.5,
            "lines.linewidth": 1.8,
            "lines.solid_capstyle": "round",
        }
    )


def load(log_dir, name):
    path = os.path.join(log_dir, name + ".csv")
    if not os.path.exists(path):
        sys.exit(f"missing log: {path}\nrun firmware/sil first")
    cols = {}
    with open(path, newline="") as fh:
        reader = csv.DictReader(fh)
        for field in reader.fieldnames:
            cols[field] = []
        for row in reader:
            for field, value in row.items():
                cols[field].append(float(value))
    return cols


def tidy(ax):
    """Only the axes a reader needs. Boxed plots add ink and no information."""
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.set_axisbelow(True)


def state_bands(ax, log):
    """Shade the flight state behind the data, so the timeline needs no second
    axis. Two y-scales on one plot is the single most misread chart there is."""
    t, state = log["t"], [int(v) for v in log["state"]]
    start, current = t[0], state[0]
    seen = []
    for i in range(1, len(t)):
        if state[i] != current:
            ax.axvspan(start, t[i], color=STATUS[STATE_NAMES[current]], alpha=0.14, lw=0)
            seen.append(current)
            start, current = t[i], state[i]
    ax.axvspan(start, t[-1], color=STATUS[STATE_NAMES[current]], alpha=0.14, lw=0)
    seen.append(current)
    return [s for i, s in enumerate(seen) if s not in seen[:i]]


def fig_attitude(log_dir, out_dir):
    """Estimate against truth through a commanded step."""
    log = load(log_dir, "roll_step")
    fig, ax = plt.subplots(figsize=(8.4, 3.6))

    ax.plot(log["t"], log["roll_true"], color=BLUE, label="true roll")
    ax.plot(log["t"], log["roll_est"], color=AMBER, ls="--", label="EKF estimate")
    ax.axhline(15.0, color=INK_MUTED, lw=1.0, ls=":", zorder=1)
    ax.annotate(
        "15 deg commanded at t = 8 s",
        xy=(8.0, 15.0),
        xytext=(9.0, 22.5),
        color=INK_MUTED,
        fontsize=8.5,
        arrowprops=dict(arrowstyle="-", color=INK_MUTED, lw=0.9),
    )

    ax.set_title("Roll step response, estimate against truth")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("roll (deg)")
    ax.legend(loc="lower right", ncol=2)
    tidy(ax)
    fig.tight_layout()
    save(fig, out_dir, "sil-attitude.png")


def fig_estimator(log_dir, out_dir):
    """The case for carrying gyro bias as a state, in one picture."""
    log = load(log_dir, "gyro_bias")
    fig, (top, bottom) = plt.subplots(
        2, 1, figsize=(8.4, 5.0), sharex=True, gridspec_kw={"height_ratios": [1.15, 1]}
    )

    top.axhline(3.0, color=INK_MUTED, lw=1.0, ls=":", zorder=1)
    top.plot(log["t"], log["bias_p_est"], color=TEAL, label="EKF bias estimate")
    top.text(
        log["t"][-1], 3.0, "  true bias 3.0 deg/s", va="center", color=INK_MUTED, fontsize=8.5
    )
    top.set_title("Gyro bias is estimated, not fought")
    top.set_ylabel("roll gyro bias (deg/s)")
    top.legend(loc="lower right")
    tidy(top)

    err = [e - t for e, t in zip(log["roll_est"], log["roll_true"])]
    bottom.axhline(0.0, color=GRID, lw=1.2, zorder=1)
    bottom.plot(log["t"], err, color=BLUE, label="estimate minus truth")
    bottom.fill_between(log["t"], err, color=BLUE, alpha=0.10, lw=0)
    bottom.set_title("Attitude error stays bounded while the bias is absorbed")
    bottom.set_xlabel("time (s)")
    bottom.set_ylabel("roll error (deg)")
    bottom.legend(loc="upper right")
    tidy(bottom)

    fig.tight_layout()
    save(fig, out_dir, "sil-estimator.png")


def fig_failsafe(log_dir, out_dir):
    """What the airframe actually did when the link went away."""
    log = load(log_dir, "signal_loss")
    fig, (top, bottom) = plt.subplots(
        2, 1, figsize=(8.4, 5.0), sharex=True, gridspec_kw={"height_ratios": [1.2, 1]}
    )

    shown = state_bands(top, log)
    top.plot(log["t"], log["z"], color=BLUE, label="altitude")
    top.set_title("Receiver lost at t = 8 s: a controlled descent, not a power cut")
    top.set_ylabel("altitude (m)")
    handles = [Patch(facecolor=STATUS[STATE_NAMES[s]], alpha=0.5, label=STATE_NAMES[s]) for s in shown]
    top.legend(
        handles=[plt.Line2D([], [], color=BLUE, label="altitude")] + handles,
        loc="upper right",
        ncol=2,
    )
    tidy(top)

    state_bands(bottom, log)
    bottom.plot(log["t"], log["throttle_cmd"], color=MAGENTA, label="commanded throttle")
    bottom.set_title("Throttle is ramped down rather than stepped to zero")
    bottom.set_xlabel("time (s)")
    bottom.set_ylabel("throttle (0 to 1)")
    bottom.legend(loc="upper right")
    tidy(bottom)

    fig.tight_layout()
    save(fig, out_dir, "sil-failsafe.png")


def fig_gain_fix(log_dir, out_dir):
    """The regression that motivated retuning, and the result."""
    legacy = load(log_dir, "legacy_gains_rejected")
    tuned = load(log_dir, "hover")

    fig, (left, right) = plt.subplots(1, 2, figsize=(9.6, 3.8), sharey=True)
    motors = [("pwm_fl", "FL", BLUE), ("pwm_fr", "FR", AMBER),
              ("pwm_bl", "BL", MAGENTA), ("pwm_br", "BR", TEAL)]

    for ax, log, title in (
        (left, legacy, "Shipped gains: Kp 1.2, Ki 0.04, Kd 15"),
        (right, tuned, "Derived gains: Kp 2.7, Ki 1.8, Kd 0.55"),
    ):
        window = [i for i, t in enumerate(log["t"]) if 6.0 <= t <= 7.5]
        for key, label, colour in motors:
            ax.plot(
                [log["t"][i] for i in window],
                [log[key][i] for i in window],
                color=colour,
                lw=1.4,
                label=label,
            )
        ax.axhline(1000, color=INK_MUTED, lw=0.9, ls=":")
        ax.axhline(2000, color=INK_MUTED, lw=0.9, ls=":")
        ax.set_title(title)
        ax.set_xlabel("time (s)")
        tidy(ax)

    left.set_ylabel("ESC command (us)")
    left.set_ylim(950, 2050)
    right.text(7.52, 2000, " ESC ceiling", color=INK_MUTED, fontsize=8, va="center")
    right.text(7.52, 1000, " ESC floor", color=INK_MUTED, fontsize=8, va="center")
    right.legend(loc="center left", ncol=4, title="motor", bbox_to_anchor=(0.0, 0.72))
    fig.suptitle(
        "Same airframe, same disturbance: the old gains chatter between the ESC stops",
        fontsize=11,
        color=INK,
        fontweight="semibold",
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    save(fig, out_dir, "sil-gain-fix.png")


def save(fig, out_dir, name):
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs", default="firmware/sil/logs")
    ap.add_argument("--out", default="assets")
    args = ap.parse_args()

    style()
    print("plotting flight logs")
    fig_attitude(args.logs, args.out)
    fig_estimator(args.logs, args.out)
    fig_failsafe(args.logs, args.out)
    fig_gain_fix(args.logs, args.out)


if __name__ == "__main__":
    main()
