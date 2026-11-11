#!/usr/bin/env python3
"""Capicola's profile stream (the bench image: cmake -DCAPICOLA_BENCH_LOG=ON).

The bench image gives the front USB to a plain serial port and writes one CSV row
every 50 ms: each stage's share of a block's real time, the engine's counts since
the last row, and the controls. See ProfileTick in src/main.cpp for the columns.

    tools/profile.py capture [-o run.csv] [--port /dev/ttyACM0] [--seconds N]
    tools/profile.py report run.csv [more.csv ...]
    tools/profile.py bench [--port ...]               # the boot benchmark's results
    tools/profile.py flash build-arm/capicola.bin     # BOOT over serial, then dfu-util

capture adds host_time (seconds since capture start) as the first column and keeps
writing until Ctrl-C (or --seconds); the boot benchmark's rows go to <output>.bench.
report prints where the time goes and what the load depends on.
"""
import argparse
import csv
import glob
import math
import statistics
import subprocess
import sys
import time

LOAD = ("total", "total_max", "feedback", "cap_l", "render_l", "cap_r", "render_r",
        "align", "post", "cap_l_max", "render_l_max", "cap_r_max", "render_r_max")
STAGES = ("feedback", "cap_l", "render_l", "cap_r", "render_r", "align", "post")
COUNTS = ("kf_l", "kf_r", "det_l", "det_r", "kept_l", "kept_r", "punch_l", "punch_r",
          "follow_l", "follow_r", "splice_l", "splice_r", "front_l", "front_r", "back_l", "back_r")
CONTROLS_K = ("pitch", "stretch", "thresh", "mix", "feedback_amt", "quality")   # x1000 on the wire
# (as src/main.cpp's ProfileTick writes them; its own header, when it comes, wins)
COLUMNS = ("t_ms,blocks,budget,dropped,total,total_max,feedback,cap_l,render_l,cap_r,render_r,"
           "align,post,cap_l_max,render_l_max,cap_r_max,render_r_max,"
           "kf_l,kf_r,det_l,det_r,kept_l,kept_r,punch_l,punch_r,follow_l,follow_r,"
           "splice_l,splice_r,front_l,front_r,back_l,back_r,silent_l,silent_r,"
           "pitch,stretch,thresh,grain,mix,feedback_amt,fade,quality,"
           "cv0,cv1,cv2,cv3,cv4,cv5").split(",")
BENCH = ("case,blocks,budget,total,total_max,feedback,cap_l,render_l,cap_r,render_r,"
         "align,post,cap_l_max,render_l_max,cap_r_max,render_r_max").split(",")


def find_port():
    ports = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        sys.exit("no /dev/ttyACM* (or usbmodem) port: is the bench image running? (--port)")
    return ports[0]


def capture(args):
    import serial
    port = args.port or find_port()
    out = open(args.output, "w", newline="")
    w = csv.writer(out)
    w.writerow(["host_time"] + COLUMNS)
    bench = {}
    rows = 0
    t0 = time.time()
    print(f"capturing {port} -> {args.output} (Ctrl-C to stop)", file=sys.stderr)
    try:
        with serial.Serial(port, 115200, timeout=1) as s:
            buf = b""
            while args.seconds is None or time.time() - t0 < args.seconds:
                buf += s.read(4096)
                *lines, buf = buf.split(b"\n")
                for ln in lines:
                    f = ln.decode("ascii", "replace").strip().split(",")
                    if not f or not f[0]:
                        continue
                    if f[0] == "t_ms":
                        if f != COLUMNS:
                            sys.exit("the firmware's columns differ from this tool's: update COLUMNS")
                        continue
                    if f[0] == "#bench":
                        bench[f[1]] = f[1:]
                        continue
                    if not f[0].isdigit() or len(f) != len(COLUMNS):
                        continue
                    w.writerow([f"{time.time() - t0:.3f}"] + f)
                    rows += 1
                    if rows % 200 == 0:
                        out.flush()
                        print(f"\r{rows} rows, {time.time() - t0:.0f} s", end="", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    out.close()
    if bench:
        with open(args.output + ".bench", "w", newline="") as bf:
            bw = csv.writer(bf)
            bw.writerow(BENCH)
            for v in bench.values():
                bw.writerow(v)
    print(f"\n{rows} rows written to {args.output}", file=sys.stderr)


def show_bench(rows):
    print(f"{'case':14s} {'total':>7s} {'worst':>7s} {'cap_l':>7s} {'rend_l':>7s} {'cap_r':>7s} "
          f"{'rend_r':>7s} {'post':>6s} {'fb':>6s}   worst cap/render block L R  (% of real time)")
    for r in rows:
        v = {k: (float(x) / 100.0 if k not in ("case", "blocks", "budget") else x) for k, x in zip(BENCH, r)}
        print(f"{v['case']:14s} {v['total']:7.2f} {v['total_max']:7.2f} {v['cap_l']:7.2f} {v['render_l']:7.2f} "
              f"{v['cap_r']:7.2f} {v['render_r']:7.2f} {v['post']:6.2f} {v['feedback']:6.2f}   "
              f"{v['cap_l_max']:.1f}/{v['render_l_max']:.1f} {v['cap_r_max']:.1f}/{v['render_r_max']:.1f}")


def bench(args):
    import serial
    port = args.port or find_port()
    seen = {}
    micro = {}
    t0 = time.time()
    with serial.Serial(port, 115200, timeout=1) as s:
        buf = b""
        while time.time() - t0 < 10 and len(seen) < 5:
            buf += s.read(4096)
            *lines, buf = buf.split(b"\n")
            for ln in lines:
                f = ln.decode("ascii", "replace").strip().split(",")
                if f and f[0] == "#bench":
                    seen[f[1]] = f[1:]
                if f and f[0] == "#micro" and len(f) >= 3:
                    micro[f[1]] = int(f[2]) / 100.0
    if not seen:
        sys.exit("no #bench rows in 10 s (is the bench image running?)")
    show_bench(seen.values())
    if micro:
        print("\nper sample, one component alone (core cycles; a sample's real time is "
              f"{int(seen[next(iter(seen))][2]) // 64} cycles)")
        for k, v in micro.items():
            print(f"  {k:14s} {v:8.1f}")


def load_rows(paths):
    rows = []
    for p in paths:
        with open(p, newline="") as f:
            for r in csv.DictReader(f):
                try:
                    d = {k: float(v) for k, v in r.items() if k}
                except ValueError:
                    continue
                for k in LOAD:
                    d[k] /= 100.0          # 0.01 % -> %
                for k in CONTROLS_K:
                    d[k] /= 1000.0
                rows.append(d)
    return rows


def pct(xs, q):
    xs = sorted(xs)
    if not xs:
        return float("nan")
    i = min(len(xs) - 1, max(0, int(math.ceil(q * len(xs))) - 1))
    return xs[i]


def report(args):
    rows = load_rows(args.files)
    if not rows:
        sys.exit("no rows")
    ts = [r["t_ms"] for r in rows]
    step = statistics.median([b - a for a, b in zip(ts, ts[1:]) if b > a] or [50.0])
    span = len(rows) * step / 1000.0   # (rows come every ~64 ms: the panel's frames)
    print(f"{len(rows)} rows, ~{span:.0f} s; block budget {rows[0]['budget']:.0f} cycles; "
          f"dropped rows {sum(r['dropped'] for r in rows[-1:]):.0f}")

    tot = [r["total"] for r in rows]
    mx = [r["total_max"] for r in rows]
    print(f"\nAUDIO LOAD (% of real time)   mean {statistics.mean(tot):5.1f}   "
          f"p95 {pct(tot, .95):5.1f}   worst 50 ms mean {max(tot):5.1f}   worst block {max(mx):5.1f}")

    print("\nWHERE IT GOES (mean %, share of the total, p95 of the stage, worst block)")
    tmean = statistics.mean(tot) or 1.0
    for k in STAGES:
        xs = [r[k] for r in rows]
        worst = max(r.get(k + "_max", 0.0) for r in rows) if (k + "_max") in rows[0] else float("nan")
        print(f"  {k:10s} {statistics.mean(xs):6.2f} %  {100 * statistics.mean(xs) / tmean:5.1f} % of it"
              f"   p95 {pct(xs, .95):6.2f}   worst block {worst:6.2f}")

    print("\nEVENTS (per second, both channels)")
    for a, b in (("kf_l", "kf_r"), ("det_l", "det_r"), ("kept_l", "kept_r"), ("punch_l", "punch_r"),
                 ("follow_l", "follow_r"), ("splice_l", "splice_r"), ("front_l", "front_r"),
                 ("back_l", "back_r")):
        print(f"  {a[:-2]:7s} L {sum(r[a] for r in rows) / span:9.1f}   R {sum(r[b] for r in rows) / span:9.1f}")
    sil = sum(r["silent_l"] + r["silent_r"] for r in rows) / (2 * len(rows))
    print(f"  silent  {100 * sil:.1f} % of the time")

    print("\nWHAT THE LOAD DEPENDS ON (mean total % by setting; rows per bin)")
    def binned(key, edges, fmt):
        bins = {}
        for r in rows:
            v = r[key]
            lab = next((fmt(e) for e in edges if v <= e), fmt(edges[-1]) + "+")
            bins.setdefault(lab, []).append(r["total"])
        cells = "  ".join(f"{lab}: {statistics.mean(v):5.1f} ({len(v)})" for lab, v in bins.items())
        print(f"  {key:8s} {cells}")
    binned("grain", [64, 128, 256, 512, 1024, 2048, 4096], lambda e: f"<={e}")
    binned("quality", [0.002, 0.005, 0.01, 0.02, 0.04, 0.08], lambda e: f"<={e:g}")
    binned("pitch", [-1, -0.5, 0, 0.5, 1, 1.5, 2, 4], lambda e: f"<={e:g}")
    binned("stretch", [-0.5, 0, 0.1, 0.5, 0.9, 1], lambda e: f"<={e:g}")

    print("\nWHAT THE LOAD MOVES WITH (correlation of the 50 ms total with each count or control)")
    def corr(xs, ys):
        mx_, my = statistics.mean(xs), statistics.mean(ys)
        sx = math.sqrt(sum((x - mx_) ** 2 for x in xs))
        sy = math.sqrt(sum((y - my) ** 2 for y in ys))
        return sum((x - mx_) * (y - my) for x, y in zip(xs, ys)) / (sx * sy) if sx and sy else 0.0
    cands = list(COUNTS) + ["grain", "quality", "pitch", "stretch", "fade", "feedback_amt"]
    cs = sorted(((corr([r[c] for r in rows], tot), c) for c in cands), key=lambda t: -abs(t[0]))
    print("  " + "   ".join(f"{c} {v:+.2f}" for v, c in cs[:10]))


def flash(args):
    import serial
    port = args.port or find_port()
    cmd = ["dfu-util", "-w", "-a", "0", "-s", "0x90040000:leave", "-D", args.image, "-d", ",0483:df11"]
    # dfu-util waits for the bootloader first; the bench image's BOOT then reboots
    # into it (it waits there for the flash). Claiming it the instant it enumerates
    # can fail LIBUSB_ERROR_BUSY on Linux: retry.
    print(f"asking {port} to reboot into the bootloader", file=sys.stderr)
    with serial.Serial(port, 115200, timeout=0.5) as s:
        s.write(b"BOOT")
        s.flush()
        time.sleep(1.0)   # (closing at once can drop the bytes)
    time.sleep(2)
    for attempt in range(5):
        if subprocess.call(cmd, timeout=60) == 0:
            return
        time.sleep(2)
    sys.exit("dfu-util failed: hold B3 while powering on, then run dfu-util by hand")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture")
    c.add_argument("-o", "--output", default=time.strftime("profile-%Y%m%d-%H%M%S.csv"))
    c.add_argument("--port")
    c.add_argument("--seconds", type=float)
    r = sub.add_parser("report")
    r.add_argument("files", nargs="+")
    b = sub.add_parser("bench")
    b.add_argument("--port")
    f = sub.add_parser("flash")
    f.add_argument("image")
    f.add_argument("--port")
    a = ap.parse_args()
    {"capture": capture, "report": report, "bench": bench, "flash": flash}[a.cmd](a)


if __name__ == "__main__":
    main()
