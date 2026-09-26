#!/usr/bin/env python3
"""Find the pulse->angle map for leg v2, one joint at a time, without stalling anything.

The leg v2 Fusion assembly carries no user parameters, so nothing in the model defines
joint zero or which way a joint moves as the pulse width rises.  This finds both by
experiment.

The safety property that makes it safe to run against an unknown mechanism:
EVERY command energises for a fixed dwell and then de-energises.  A servo that has been
driven into an end stop stops pushing after `--dwell` seconds whatever happens next --
including if the operator says nothing, the SSH session drops, or this process is killed.
Stalling is the damage mode; a stall that cannot outlive one dwell cannot cook a servo.

De-energising is NOT the same as safe: a de-energised leg does not hold its pose (measured
2026-09-23, it went flat).  Support the leg so a collapse is harmless before running this.

  probe   one pulse width, one dwell, then off       -- "where does 1500 put this joint?"
  sweep   a series of pulse widths, dwell at each    -- "which way does rising us go?"
  hold    energise and stay energised                -- for a load test at a known pose

Usage:
  ./leg-calibrate.py probe --ch 3 --us 1500
  ./leg-calibrate.py sweep --ch 3 --from 1400 --to 1600 --step 25
  ./leg-calibrate.py hold  --ch 2 --us 1480 --seconds 20
  ./leg-calibrate.py off
"""
import argparse, importlib.util, json, os, signal, sys, time

SB_PATH = os.environ.get("SERVO_BENCH", "/home/ben/servo-bench.py")
spec = importlib.util.spec_from_file_location("sb", SB_PATH)
sb = importlib.util.module_from_spec(spec); spec.loader.exec_module(sb)

HZ         = 200.0
BUS, ADDR  = 1, 0x40
ALLCALL    = 0x70
NAMES      = {0: "coxa yaw", 1: "hip lift", 2: "femur joint", 3: "tibia joint"}
# Deliberately narrow.  270 deg servos span 500-2500 us; this is the middle ~15%, so a
# blind first energise moves a joint tens of degrees, not into an end stop.
SAFE_MIN, SAFE_MAX = 1200, 1800
RECORD     = os.path.expanduser("~/leg-calibration.json")


class Leg:
    def __init__(self):
        self.p = sb.Pca9685(BUS, ADDR, 25_000_000)
        _pre, self.hz = self.p.init(HZ)

    def energise(self, ch, us):
        self.p.set_us(ch, us, self.hz)

    def off(self, ch=None):
        # all_off() LAST. It writes the ALL_LED registers, which the chip copies into
        # every channel; a per-channel write afterwards overwrites that and re-enables
        # the output. The original order did exactly that -- it undid its own stop.
        if ch is None:
            for c in range(16):
                self.p.channel_off(c)
            self.p.all_off()
        else:
            self.p.channel_off(ch)

    def estop(self):
        """Both mechanisms: every channel off, then the all-call stop at 0x70."""
        try:
            self.off()
        finally:
            try:
                self.p.b.write_byte_data(ALLCALL, 0xFD, 0x10)
            except Exception as e:
                print(f"  ! all-call stop failed: {e}", file=sys.stderr)

    def mode1(self):
        return self.p.rd(0x00)


def clamp(us, allow_wide):
    lo, hi = (500, 2500) if allow_wide else (SAFE_MIN, SAFE_MAX)
    c = max(lo, min(hi, int(us)))
    if c != int(us):
        print(f"  ! {int(us)} us clamped to {c} (limit {lo}-{hi}; --wide to widen)")
    return c


def pulse(leg, ch, us, dwell):
    """Energise for exactly `dwell` seconds, then de-energise.  Always de-energises."""
    print(f"  ch{ch} {NAMES.get(ch,'?'):12s} <- {us} us  for {dwell:.2f}s", flush=True)
    try:
        leg.energise(ch, us)
        time.sleep(dwell)
    finally:
        leg.off(ch)
    m = leg.mode1()
    if m & 0x10:
        print(f"  !! MODE1 0x{m:02x} SLEEP SET -- the board browned out. Stop and check the rail.")
    return m


def record(entry):
    data = []
    if os.path.exists(RECORD):
        try:
            data = json.load(open(RECORD))
        except Exception:
            pass
    data.append(entry)
    json.dump(data, open(RECORD, "w"), indent=1)


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("probe", "sweep", "hold"):
        s = sub.add_parser(name)
        s.add_argument("--ch", type=int, required=True)
        s.add_argument("--dwell", type=float, default=0.5)
        s.add_argument("--wide", action="store_true", help="allow the full 500-2500 us range")
        if name == "probe":
            s.add_argument("--us", type=int, required=True)
        if name == "sweep":
            s.add_argument("--from", dest="lo", type=int, required=True)
            s.add_argument("--to", dest="hi", type=int, required=True)
            s.add_argument("--step", type=int, default=25)
            s.add_argument("--gap", type=float, default=0.35, help="de-energised pause between steps")
        if name == "hold":
            s.add_argument("--us", type=int, required=True)
            s.add_argument("--seconds", type=float, default=15.0)
    sub.add_parser("off")
    a = ap.parse_args()

    leg = Leg()
    signal.signal(signal.SIGINT,  lambda *_: (leg.estop(), print("\n  aborted -- all channels off"), sys.exit(130)))
    signal.signal(signal.SIGTERM, lambda *_: (leg.estop(), sys.exit(143)))

    print(f"PCA9685 @0x{ADDR:02x}  {leg.hz:.2f} Hz  MODE1 0x{leg.mode1():02x}")

    if a.cmd == "off":
        leg.estop(); print("  all 16 channels off + all-call stop"); return

    if a.cmd == "probe":
        us = clamp(a.us, a.wide)
        pulse(leg, a.ch, us, a.dwell)
        record({"t": time.time(), "cmd": "probe", "ch": a.ch, "us": us})

    elif a.cmd == "sweep":
        lo, hi = clamp(a.lo, a.wide), clamp(a.hi, a.wide)
        step = abs(a.step) * (1 if hi >= lo else -1)
        vals = list(range(lo, hi + (1 if step > 0 else -1), step))
        print(f"  {len(vals)} steps, {abs(step)} us apart, {a.dwell:.2f}s energised / {a.gap:.2f}s off")
        for us in vals:
            pulse(leg, a.ch, us, a.dwell)
            time.sleep(a.gap)
        record({"t": time.time(), "cmd": "sweep", "ch": a.ch, "from": lo, "to": hi, "step": step})

    elif a.cmd == "hold":
        us = clamp(a.us, a.wide)
        print(f"  ch{a.ch} {NAMES.get(a.ch,'?')} HELD at {us} us for {a.seconds:.0f}s -- Ctrl-C aborts")
        pulse(leg, a.ch, us, a.seconds)
        record({"t": time.time(), "cmd": "hold", "ch": a.ch, "us": us, "seconds": a.seconds})

    leg.off()
    print(f"  done -- channels off, MODE1 0x{leg.mode1():02x}   log: {RECORD}")


if __name__ == "__main__":
    main()
