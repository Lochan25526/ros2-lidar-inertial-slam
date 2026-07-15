#!/usr/bin/env python3
"""
ros2_resmon - CPU/RAM profiler for ROS 2 nodes.

Standalone tool (no ROS dependency). It reads /proc via psutil, matching
target processes by cmdline substring, so it works for both C++ and Python
nodes. Two subcommands:

    record    profile an already-running workspace for a while
    compare   combine two recorded runs into a table + grayscale charts

Target platform: Ubuntu 22.04 / Raspberry Pi 4 / ROS 2 Humble.

Dependencies:
    pip install psutil matplotlib
"""

import argparse
import csv
import os
import statistics
import sys
import time
from collections import defaultdict

import psutil


# --------------------------------------------------------------------------
# Presets: substrings to look for in a process cmdline.
# A process may be C++ or Python, so we match the executable/argv, not comm.
# --------------------------------------------------------------------------
PRESETS = {
    "lidar_slam": [
        "rplidar",
        "sllidar",
        "rf2o",
        "imu_filter_madgwick",
        "madgwick",
        "ekf_node",
        "robot_localization",
        "slam_toolbox",
        "icm20948",
        "robot_state_publisher",
    ],
    "orbslam": [
        "orbslam3_node",
        "orbslam3_vio",
        "camera",
        "libcamera",
        "cam2image",
        "icm20948",
    ],
}

# Map a matched substring -> friendly report-row name.
# Anything not listed falls back to the substring itself.
FRIENDLY = {
    "rf2o": "RF2O",
    "slam_toolbox": "SLAM Toolbox",
    "orbslam3_node": "ORB-SLAM3 mono",
    "orbslam3_vio": "ORB-SLAM3 VIO",
    "ekf_node": "EKF",
    "imu_filter_madgwick": "Madgwick",
    "madgwick": "Madgwick",
    "rplidar": "RPLIDAR driver",
    "sllidar": "RPLIDAR driver",
    "camera": "Camera node",
    "libcamera": "Camera node",
}

TOTAL_ROW = "TOTAL (workspace)"
THERMAL_ZONE = "/sys/class/thermal/thermal_zone0/temp"


def friendly_name(substr):
    return FRIENDLY.get(substr, substr)


def read_soc_temp():
    """Raspberry Pi SoC temperature in degrees C, or None if unreadable."""
    try:
        with open(THERMAL_ZONE) as f:
            return int(f.read().strip()) / 1000.0
    except (OSError, ValueError):
        return None


# --------------------------------------------------------------------------
# record mode
# --------------------------------------------------------------------------
def discover(matchers, exclude_pid):
    """Return {pid: friendly_label} for every process whose cmdline (or name)
    contains one of the match substrings."""
    found = {}
    for p in psutil.process_iter(["pid", "cmdline", "name"]):
        try:
            pid = p.info["pid"]
            if pid == exclude_pid:
                continue
            cmd = " ".join(p.info["cmdline"] or [])
            if not cmd:
                cmd = p.info["name"] or ""
            low = cmd.lower()
            for sub in matchers:
                if sub.lower() in low:
                    found[pid] = friendly_name(sub)
                    break
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return found


def cmd_record(args):
    if args.match:
        matchers = [m.strip() for m in args.match.split(",") if m.strip()]
    else:
        if args.preset not in PRESETS:
            sys.exit(
                "error: --preset must be one of {} (or use --match)".format(
                    ", ".join(PRESETS)
                )
            )
        matchers = PRESETS[args.preset]

    os.makedirs(args.out, exist_ok=True)
    ncpu = psutil.cpu_count() or 1
    self_pid = os.getpid()

    # Warm-up: the first cpu_percent() call always returns 0.0.
    psutil.cpu_percent(None)

    # Persistent Process objects so cpu_percent() measures the delta over each
    # interval. Newly seen PIDs are primed (their first sample reads 0.0).
    proc_cache = {}  # pid -> psutil.Process

    initial = discover(matchers, self_pid)
    print("=" * 70)
    print("ros2_resmon record  |  label={}  preset/match={}".format(
        args.label, args.match or args.preset))
    print("cores={}  duration={}s  interval={}s  out={}".format(
        ncpu, args.duration, args.interval, args.out))
    print("-" * 70)
    if initial:
        print("Matched {} process(es):".format(len(initial)))
        for pid, label in sorted(initial.items()):
            print("  PID {:>7}  ->  {}".format(pid, label))
    else:
        print("WARNING: no matching processes found yet. Is the workspace running?")
        print("Matchers: {}".format(", ".join(matchers)))
    print("=" * 70)

    samples = []  # per-(interval, pid) rows
    start = time.time()
    deadline = start + args.duration
    next_tick = start

    while True:
        now = time.time()
        if now >= deadline:
            break

        ts = now
        elapsed = ts - start
        sys_cpu = psutil.cpu_percent(None)
        sys_mem_used = psutil.virtual_memory().used / (1024 * 1024)
        soc_temp = read_soc_temp()

        current = discover(matchers, self_pid)

        # Drop cache entries for PIDs that disappeared.
        for dead in [pid for pid in proc_cache if pid not in current]:
            proc_cache.pop(dead, None)

        for pid, label in current.items():
            try:
                if pid not in proc_cache:
                    proc = psutil.Process(pid)
                    proc.cpu_percent(None)  # prime new PID
                    proc_cache[pid] = proc
                proc = proc_cache[pid]

                cpu_raw = proc.cpu_percent(None)
                cpu_norm = cpu_raw / ncpu
                rss_mb = proc.memory_info().rss / (1024 * 1024)
                threads = proc.num_threads()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                proc_cache.pop(pid, None)
                continue

            samples.append({
                "ts": ts,
                "elapsed": round(elapsed, 3),
                "label": label,
                "pid": pid,
                "cpu_raw": cpu_raw,
                "cpu_norm": cpu_norm,
                "rss_mb": rss_mb,
                "threads": threads,
                "sys_cpu": sys_cpu,
                "sys_mem_used_mb": sys_mem_used,
                "soc_temp_c": soc_temp,
            })

        # Sleep until the next interval boundary (drift-free-ish).
        next_tick += args.interval
        sleep_for = next_tick - time.time()
        if sleep_for > 0:
            time.sleep(min(sleep_for, args.interval))

    if not samples:
        print("\nNo samples recorded (no matching processes during the run).")
        # Still emit empty files so downstream tooling has something to read.
    ts_path = os.path.join(args.out, "{}_timeseries.csv".format(args.label))
    sum_path = os.path.join(args.out, "{}_summary.csv".format(args.label))

    write_timeseries(ts_path, samples)
    summary = build_summary(samples)
    write_summary(sum_path, summary)

    print("\nWrote:")
    print("  {}".format(ts_path))
    print("  {}".format(sum_path))
    print_summary_table(args.label, summary)


def write_timeseries(path, samples):
    cols = ["ts", "elapsed", "label", "pid", "cpu_raw", "cpu_norm",
            "rss_mb", "threads", "sys_cpu", "sys_mem_used_mb", "soc_temp_c"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for s in samples:
            row = dict(s)
            for k in ("cpu_raw", "cpu_norm", "rss_mb", "sys_cpu", "sys_mem_used_mb"):
                row[k] = round(row[k], 3)
            if row["soc_temp_c"] is not None:
                row["soc_temp_c"] = round(row["soc_temp_c"], 2)
            w.writerow(row)


def _agg(values):
    """(mean, peak, std) for a list of numbers; zeros for empty."""
    if not values:
        return 0.0, 0.0, 0.0
    mean = statistics.fmean(values)
    peak = max(values)
    std = statistics.pstdev(values) if len(values) > 1 else 0.0
    return mean, peak, std


def build_summary(samples):
    """Aggregate per node (label). Multiple PIDs that share a label are summed
    per sample-timestamp before aggregating. The TOTAL row sums all nodes per
    timestamp, then aggregates across timestamps."""
    # label -> ts -> [cpu_raw, cpu_norm, rss]
    per = defaultdict(lambda: defaultdict(lambda: [0.0, 0.0, 0.0]))
    total = defaultdict(lambda: [0.0, 0.0, 0.0])  # ts -> [cpu_raw, cpu_norm, rss]

    for s in samples:
        acc = per[s["label"]][s["ts"]]
        acc[0] += s["cpu_raw"]
        acc[1] += s["cpu_norm"]
        acc[2] += s["rss_mb"]
        tacc = total[s["ts"]]
        tacc[0] += s["cpu_raw"]
        tacc[1] += s["cpu_norm"]
        tacc[2] += s["rss_mb"]

    rows = []
    for label in sorted(per):
        series = list(per[label].values())
        rows.append(_summary_row(label, series))

    if total:
        rows.append(_summary_row(TOTAL_ROW, list(total.values())))
    return rows


def _summary_row(label, series):
    cpu_raw = [v[0] for v in series]
    cpu_norm = [v[1] for v in series]
    rss = [v[2] for v in series]
    r_mean, r_peak, r_std = _agg(cpu_raw)
    n_mean, n_peak, n_std = _agg(cpu_norm)
    rss_mean, rss_peak, _ = _agg(rss)
    return {
        "label": label,
        "cpu_raw_mean": r_mean,
        "cpu_raw_peak": r_peak,
        "cpu_raw_std": r_std,
        "cpu_norm_mean": n_mean,
        "cpu_norm_peak": n_peak,
        "cpu_norm_std": n_std,
        "rss_mean_mb": rss_mean,
        "rss_peak_mb": rss_peak,
        "samples": len(series),
    }


SUMMARY_COLS = ["label", "cpu_raw_mean", "cpu_raw_peak", "cpu_raw_std",
                "cpu_norm_mean", "cpu_norm_peak", "cpu_norm_std",
                "rss_mean_mb", "rss_peak_mb", "samples"]


def write_summary(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=SUMMARY_COLS)
        w.writeheader()
        for r in rows:
            out = dict(r)
            for k in SUMMARY_COLS:
                if k not in ("label", "samples"):
                    out[k] = round(out[k], 3)
            w.writerow(out)


def print_summary_table(label, rows):
    print("\n" + "=" * 70)
    print("SUMMARY: {}".format(label))
    print("=" * 70)
    hdr = "{:<22} {:>9} {:>9} {:>9} {:>10}".format(
        "Node", "CPU% mean", "CPU% pk", "CPU%n mn", "RAM MB")
    print(hdr)
    print("-" * 70)
    for r in rows:
        sep = r["label"] == TOTAL_ROW
        if sep:
            print("-" * 70)
        print("{:<22} {:>9.1f} {:>9.1f} {:>9.1f} {:>10.1f}".format(
            r["label"][:22], r["cpu_raw_mean"], r["cpu_raw_peak"],
            r["cpu_norm_mean"], r["rss_mean_mb"]))
    print("=" * 70)


# --------------------------------------------------------------------------
# compare mode
# --------------------------------------------------------------------------
def _pretty_workspace(path):
    base = os.path.basename(path)
    for suffix in ("_summary.csv", ".csv"):
        if base.endswith(suffix):
            base = base[: -len(suffix)]
            break
    mapping = {
        "lidar_slam": "LiDAR SLAM",
        "orbslam": "ORB-SLAM",
        "orb_slam": "ORB-SLAM",
    }
    return mapping.get(base, base.replace("_", " "))


def read_summary_csv(path):
    """Return (nodes, total) where nodes is a list of dicts (excluding the
    TOTAL row) and total is that TOTAL row (or None)."""
    nodes, total = [], None
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            rec = {
                "label": row["label"],
                "cpu_mean": float(row.get("cpu_raw_mean", 0) or 0),
                "cpu_peak": float(row.get("cpu_raw_peak", 0) or 0),
                "ram_mean": float(row.get("rss_mean_mb", 0) or 0),
            }
            if rec["label"] == TOTAL_ROW:
                total = rec
            else:
                nodes.append(rec)
    return nodes, total


def cmd_compare(args):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(args.out, exist_ok=True)
    ws_a = _pretty_workspace(args.a)
    ws_b = _pretty_workspace(args.b)
    nodes_a, total_a = read_summary_csv(args.a)
    nodes_b, total_b = read_summary_csv(args.b)

    # ---- comparison_table.csv ----
    table_path = os.path.join(args.out, "comparison_table.csv")
    with open(table_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Workspace", "Node", "CPU% mean", "CPU% peak", "RAM MB mean"])
        for rec in nodes_a:
            w.writerow([ws_a, rec["label"], round(rec["cpu_mean"], 1),
                        round(rec["cpu_peak"], 1), round(rec["ram_mean"], 1)])
        for rec in nodes_b:
            w.writerow([ws_b, rec["label"], round(rec["cpu_mean"], 1),
                        round(rec["cpu_peak"], 1), round(rec["ram_mean"], 1)])
        if total_a:
            w.writerow([ws_a, TOTAL_ROW, round(total_a["cpu_mean"], 1),
                        round(total_a["cpu_peak"], 1), round(total_a["ram_mean"], 1)])
        if total_b:
            w.writerow([ws_b, TOTAL_ROW, round(total_b["cpu_mean"], 1),
                        round(total_b["cpu_peak"], 1), round(total_b["ram_mean"], 1)])

    # ---- per-node charts (grayscale, print-friendly) ----
    style_a = {"color": "0.55", "hatch": "////", "edgecolor": "black"}
    style_b = {"color": "0.85", "hatch": "....", "edgecolor": "black"}

    _bar_per_node(plt, os.path.join(args.out, "cpu_compare.png"),
                  "Mean CPU% per node", "CPU %",
                  nodes_a, nodes_b, ws_a, ws_b, style_a, style_b, "cpu_mean")
    _bar_per_node(plt, os.path.join(args.out, "cpu_peak.png"),
                  "Peak CPU% per node", "CPU %",
                  nodes_a, nodes_b, ws_a, ws_b, style_a, style_b, "cpu_peak")
    _bar_per_node(plt, os.path.join(args.out, "ram_compare.png"),
                  "Mean RAM (MB) per node", "RAM MB",
                  nodes_a, nodes_b, ws_a, ws_b, style_a, style_b, "ram_mean")

    # ---- workspace total (2-bar CPU + 2-bar RAM) ----
    _workspace_total(plt, os.path.join(args.out, "workspace_total.png"),
                     ws_a, ws_b, total_a, total_b, style_a, style_b)

    print("Wrote comparison to {}/".format(args.out))
    for name in ("comparison_table.csv", "cpu_compare.png", "cpu_peak.png",
                 "ram_compare.png", "workspace_total.png"):
        print("  {}".format(os.path.join(args.out, name)))

    _print_compare_table(ws_a, ws_b, nodes_a, nodes_b, total_a, total_b)


def _bar_per_node(plt, path, title, ylabel, nodes_a, nodes_b,
                  ws_a, ws_b, style_a, style_b, key):
    labels, values, styles = [], [], []
    for rec in nodes_a:
        labels.append(rec["label"])
        values.append(rec[key])
        styles.append(style_a)
    for rec in nodes_b:
        labels.append(rec["label"])
        values.append(rec[key])
        styles.append(style_b)

    fig, ax = plt.subplots(figsize=(max(6, len(labels) * 0.9), 4.5))
    x = range(len(labels))
    for i, (v, st) in enumerate(zip(values, styles)):
        ax.bar(i, v, width=0.7, color=st["color"], hatch=st["hatch"],
               edgecolor=st["edgecolor"], linewidth=0.8)
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels, rotation=35, ha="right", fontsize=8)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(axis="y", color="0.8", linewidth=0.6)
    ax.set_axisbelow(True)

    # Legend proxies.
    from matplotlib.patches import Patch
    handles = [
        Patch(facecolor=style_a["color"], hatch=style_a["hatch"],
              edgecolor="black", label=ws_a),
        Patch(facecolor=style_b["color"], hatch=style_b["hatch"],
              edgecolor="black", label=ws_b),
    ]
    ax.legend(handles=handles, fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)


def _workspace_total(plt, path, ws_a, ws_b, total_a, total_b, style_a, style_b):
    cpu_a = total_a["cpu_mean"] if total_a else 0.0
    cpu_b = total_b["cpu_mean"] if total_b else 0.0
    ram_a = total_a["ram_mean"] if total_a else 0.0
    ram_b = total_b["ram_mean"] if total_b else 0.0

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 4.5))
    for ax, vals, ylabel, title in (
        (ax1, (cpu_a, cpu_b), "CPU %", "Total workspace CPU"),
        (ax2, (ram_a, ram_b), "RAM MB", "Total workspace RAM"),
    ):
        ax.bar(0, vals[0], width=0.6, color=style_a["color"],
               hatch=style_a["hatch"], edgecolor="black", linewidth=0.8)
        ax.bar(1, vals[1], width=0.6, color=style_b["color"],
               hatch=style_b["hatch"], edgecolor="black", linewidth=0.8)
        ax.set_xticks([0, 1])
        ax.set_xticklabels([ws_a, ws_b], fontsize=9)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(axis="y", color="0.8", linewidth=0.6)
        ax.set_axisbelow(True)
        for i, v in enumerate(vals):
            ax.text(i, v, " {:.1f}".format(v), ha="center", va="bottom", fontsize=8)
    fig.suptitle("Workspace totals: {} vs {}".format(ws_a, ws_b))
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)


def _print_compare_table(ws_a, ws_b, nodes_a, nodes_b, total_a, total_b):
    print("\n" + "=" * 70)
    print("COMPARISON: {} vs {}".format(ws_a, ws_b))
    print("=" * 70)
    print("{:<14} {:<20} {:>9} {:>9} {:>10}".format(
        "Workspace", "Node", "CPU% mean", "CPU% pk", "RAM MB"))
    print("-" * 70)
    for ws, rec in ([(ws_a, r) for r in nodes_a] + [(ws_b, r) for r in nodes_b]):
        print("{:<14} {:<20} {:>9.1f} {:>9.1f} {:>10.1f}".format(
            ws[:14], rec["label"][:20], rec["cpu_mean"], rec["cpu_peak"],
            rec["ram_mean"]))
    print("-" * 70)
    for ws, tot in ((ws_a, total_a), (ws_b, total_b)):
        if tot:
            print("{:<14} {:<20} {:>9.1f} {:>9.1f} {:>10.1f}".format(
                ws[:14], TOTAL_ROW, tot["cpu_mean"], tot["cpu_peak"],
                tot["ram_mean"]))
    print("=" * 70)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(
        prog="ros2_resmon",
        description="CPU/RAM profiler for ROS 2 nodes (psutil + matplotlib only).")
    sub = p.add_subparsers(dest="cmd", required=True)

    rec = sub.add_parser("record", help="profile a running workspace")
    rec.add_argument("--label", required=True,
                     help="run name, e.g. lidar_slam or orbslam")
    rec.add_argument("--duration", type=float, default=60,
                     help="seconds to record (default 60)")
    rec.add_argument("--interval", type=float, default=1.0,
                     help="sample interval seconds (default 1.0)")
    rec.add_argument("--out", default="./resmon_out",
                     help="output directory (default ./resmon_out)")
    rec.add_argument("--preset", choices=sorted(PRESETS),
                     help="built-in match set: lidar_slam or orbslam")
    rec.add_argument("--match",
                     help='comma-separated cmdline substrings, overrides --preset')
    rec.set_defaults(func=cmd_record)

    cmp = sub.add_parser("compare", help="combine two recorded runs")
    cmp.add_argument("--a", required=True, help="first *_summary.csv")
    cmp.add_argument("--b", required=True, help="second *_summary.csv")
    cmp.add_argument("--out", default="./compare",
                     help="output directory (default ./compare)")
    cmp.set_defaults(func=cmd_compare)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.cmd == "record" and not args.preset and not args.match:
        sys.exit("error: record needs --preset {lidar_slam,orbslam} or --match")
    args.func(args)


if __name__ == "__main__":
    main()
