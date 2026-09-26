#!/usr/bin/env python3
"""Wiggle one PCA9685 channel back and forth so you can troubleshoot wiring hands-free.

Run it, leave it running, and go poke the suspect connector / buck / splice. The servo
sweeps between two pulse widths forever until you press Ctrl-C, so an intermittent
connection shows up as a stutter instead of needing you to time a command.

  ./exercise-ch.py                 # ch3, 1300 <-> 1700, 1.2 s each way, forever
  ./exercise-ch.py --ch 3 --lo 1200 --hi 1800 --period 0.8
  ./exercise-ch.py --once          # one there-and-back, then stop
  ./exercise-ch.py --find          # walk ch4..ch15 to locate a mis-plugged servo
  ./exercise-ch.py --off           # release everything and exit

WHAT TO MEASURE WHILE IT RUNS (DC volts on a multimeter):
  signal pin -> GND   ~0.85 V at 1300 us, ~1.15 V at 1700 us, and it should VISIBLY
                      alternate. A steady 0 V means no pulse is reaching that pin.
  servo V+   -> GND   should hold 6.0 V from the buck and NOT dip when the servo moves.
                      An LM2596 sagging under the motion surge looks exactly like a dead
                      servo -- which cost us an evening on 2026-09-25.
  GND continuity      servo ground MUST be common with the PCA9685 ground even though
                      V+ comes from the buck. A servo signal is referenced to ground; a
                      floating ground means no valid pulse and no current draw at all.

⚠ RELEASING A CHANNEL DOES NOT REMOVE TORQUE. Measured 2026-09-26: these digital servos
  keep holding position after the pulses stop -- 216 mA across three of them with every
  channel read back as FULL_OFF, and they cannot be moved by hand. The only real
  torque-off is cutting V+ at the supply.
"""
import argparse, importlib.util, os, signal, sys, time

SB = os.environ.get("SERVO_BENCH", os.path.expanduser("~/servo-bench.py"))
spec = importlib.util.spec_from_file_location("sb", SB)
sb = importlib.util.module_from_spec(spec); spec.loader.exec_module(sb)

HZ, BUS, ADDR, ALLCALL = 200.0, 1, 0x40, 0x70
LIMIT_LO, LIMIT_HI = 1000, 2000


def release(p):
    for c in range(16):
        p.channel_off(c)
    p.all_off()                                  # all_off LAST: a per-channel write after
    try:                                         # it would overwrite the stop it just set
        p.b.write_byte_data(ALLCALL, 0xFD, 0x10) # all-call ALL_LED_OFF_H = FULL_OFF
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ch", type=int, default=3)
    ap.add_argument("--lo", type=int, default=1300)
    ap.add_argument("--hi", type=int, default=1700)
    ap.add_argument("--period", type=float, default=1.2, help="seconds at each end")
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--find", action="store_true", help="walk ch4..ch15 looking for a servo")
    ap.add_argument("--off", action="store_true")
    a = ap.parse_args()

    lo, hi = sorted((max(LIMIT_LO, a.lo), min(LIMIT_HI, a.hi)))
    p = sb.Pca9685(BUS, ADDR, 25_000_000)
    _pre, hz = p.init(HZ)

    def bail(*_):
        release(p)
        print("\n  released (NOTE: the servo may still be holding -- cut V+ to go limp)")
        sys.exit(0)
    signal.signal(signal.SIGINT, bail)
    signal.signal(signal.SIGTERM, bail)

    print(f"PCA9685 @0x{ADDR:02x}  {hz:.2f} Hz  MODE1 0x{p.rd(0x00):02x}")

    if a.off:
        release(p); print("  all channels released"); return

    if a.find:
        print("  walking ch4..ch15 (skipping 0-2, the big servos) -- watch for movement")
        for c in range(4, 16):
            print(f"    ch{c:2d} ...", flush=True)
            for us in (lo, hi, 1500):
                p.set_us(c, us, hz); time.sleep(0.45)
            p.channel_off(c)
        release(p); return

    print(f"  ch{a.ch}: {lo} <-> {hi} us, {a.period}s each way. Ctrl-C to stop.")
    n = 0
    while True:
        for us in (lo, hi):
            p.set_us(a.ch, us, hz)
            n += 1
            m = p.rd(0x00)
            flag = "  !! MODE1 SLEEP SET - board brown-out" if m & 0x10 else ""
            print(f"\r  move {n:5d}   {us} us   MODE1 0x{m:02x}{flag}   ", end="", flush=True)
            time.sleep(a.period)
        if a.once:
            break
    bail()


if __name__ == "__main__":
    main()
