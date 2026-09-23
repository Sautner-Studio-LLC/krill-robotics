#!/usr/bin/env python3
"""Leg v2 shakedown: energise gently, exercise, return home, probe press direction.

Every move ramps the COMMANDED pulse in small steps, so nothing lurches except the
unavoidable first energise of each channel (a PWM servo has no feedback and no soft
start). Channels are energised ONE AT A TIME so at most one joint can snap at once.

ch0 (coxa yaw) is deliberately left limp: yawing against a planted toe is an
unintended lateral load test on a toe rated ~0.5 kg.
"""
import importlib.util, sys, time, random

spec = importlib.util.spec_from_file_location("sb", "/home/ben/servo-bench.py")
sb = importlib.util.module_from_spec(spec); spec.loader.exec_module(sb)

HZ      = 200.0
HOME    = 1500
CH      = {1: "hip lift", 2: "femur joint", 3: "tibia joint"}
ORDER   = [3, 2, 1]          # lightest / shortest reach first
MIN_US, MAX_US = 1150, 1850  # tighter than the 1000-2000 default for a shakedown

def log(msg): print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

class Leg:
    def __init__(self):
        self.p = sb.Pca9685(1, 0x40, 25_000_000)   # bus NUMBER, not an SMBus object
        _pre, self.hz = self.p.init(HZ)
        log(f"board init: prescale {_pre}, {self.hz:.2f} Hz actual")
        self.cur = {}

    def raw(self, ch, us):
        us = max(MIN_US, min(MAX_US, int(us)))
        self.p.set_us(ch, us, self.hz)
        self.cur[ch] = us
        return us

    def energise(self, ch, us):
        log(f"  energise ch{ch} ({CH[ch]}) at {us} us  <- may snap")
        self.raw(ch, us)

    def ramp(self, moves, step=2, dt=0.02):
        """moves: {ch: target}. Ramps every channel together at ~100 us/s."""
        start = {c: self.cur[c] for c in moves}
        span  = max(abs(moves[c] - start[c]) for c in moves) or 1
        n     = max(1, span // step)
        for i in range(1, n + 1):
            for c, t in moves.items():
                self.raw(c, start[c] + (t - start[c]) * i / n)
            time.sleep(dt)

    def off(self):
        self.p.all_off()
        log("all channels off (limp)")

def main():
    stage = sys.argv[1] if len(sys.argv) > 1 else "all"
    leg = Leg()
    mode1 = leg.p.rd(0x00)
    log(f"MODE1 = 0x{mode1:02x} (SLEEP bit {'SET - brownout!' if mode1 & 0x10 else 'clear'})")

    # --- A: energise, one at a time -----------------------------------------
    log("STAGE A - energising ch3, ch2, ch1 one at a time. ch0 stays limp.")
    for ch in ORDER:
        leg.energise(ch, HOME)
        time.sleep(2.5)
    log("  all three holding at 1500 us. This pose is now HOME.")
    time.sleep(2)

    # --- B: exercise ---------------------------------------------------------
    log("STAGE B - per-joint sweeps, +/-100 us (~13.5 deg) at ~13 deg/s")
    for ch in ORDER:
        log(f"  sweeping ch{ch} ({CH[ch]})")
        for tgt in (HOME + 100, HOME - 100, HOME):
            leg.ramp({ch: tgt}); time.sleep(0.6)
        time.sleep(1.0)

    log("STAGE B2 - all three wandering together, 30 s, bounded +/-80 us")
    random.seed(2)
    t0 = time.time()
    while time.time() - t0 < 30:
        leg.ramp({c: HOME + random.randint(-80, 80) for c in ORDER}, step=3)
        time.sleep(0.25)

    # --- C: home -------------------------------------------------------------
    log("STAGE C - returning to HOME (1500 all)")
    leg.ramp({c: HOME for c in ORDER}); time.sleep(2)

    if stage == "exercise":
        log("holding HOME. Done.")
        return

    # --- D: which way is 'press'? -------------------------------------------
    log("STAGE D - press-direction probe on ch1 (hip lift). WATCH THE SCALE.")
    for tgt, label in ((HOME - 60, "A: ch1 DOWN 60 us"), (HOME + 60, "B: ch1 UP 60 us")):
        log(f"  {label} - holding 5 s")
        leg.ramp({1: tgt}); time.sleep(5)
        leg.ramp({1: HOME}); time.sleep(2)
    log("HOME, holding. Which one loaded the scale, A or B?")

if __name__ == "__main__":
    main()
