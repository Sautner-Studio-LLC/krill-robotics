#!/usr/bin/env python3
"""PCA9685 servo bench tool -- direct I2C, no Krill, no build.

Runs ON the Pi. Needs python3-smbus2 and /dev/i2c-1.

The point of this tool is to be the REFERENCE: it does the register sequence by
hand so that when a higher-level driver misbehaves you have something known-good
to compare against. It deliberately has no dependencies beyond smbus2.

    ./servo-bench.py scan
    ./servo-bench.py init --freq 50
    ./servo-bench.py us 0 1500            # one channel to a pulse width
    ./servo-bench.py sweep 0 --from 1400 --to 1600 --step 20 --dwell 40
    ./servo-bench.py stop                 # all-call E-stop: every channel, every board

SAFETY
  * Pulse width is clamped to --min-us/--max-us (default 1000..2000, a
    conservative middle band). A 270 deg servo may accept 500..2500, but driving
    an unknown servo to its mechanical stop stalls it, and a stalled 80 kg servo
    draws many amps and gets hot fast. Widen only once you know the real stops.
  * Every path is interruptible and `stop` always works.
"""
import argparse, sys, time

MODE1, MODE2 = 0x00, 0x01
SUBADR1, ALLCALLADR = 0x02, 0x05
LED0_ON_L = 0x06
ALL_LED_ON_L, ALL_LED_OFF_H = 0xFA, 0xFD
PRE_SCALE, TESTMODE = 0xFE, 0xFF

M1_RESTART, M1_EXTCLK, M1_AI, M1_SLEEP, M1_ALLCALL = 0x80, 0x40, 0x20, 0x10, 0x01
M2_OUTDRV = 0x04
FULL_OFF = 0x10          # LEDn_OFF_H bit 4


class Pca9685:
    def __init__(self, bus, addr, osc_hz):
        from smbus2 import SMBus
        self.b = SMBus(bus)
        self.a = addr
        self.osc = osc_hz

    def rd(self, reg):  return self.b.read_byte_data(self.a, reg)
    def wr(self, reg, v): self.b.write_byte_data(self.a, reg, v & 0xFF)

    def identify(self):
        return {r: self.rd(v) for r, v in
                (("MODE1", MODE1), ("MODE2", MODE2), ("SUBADR1", SUBADR1),
                 ("ALLCALL", ALLCALLADR), ("PRE_SCALE", PRE_SCALE))}

    def prescale_for(self, hz):
        # datasheet: prescale = round(osc / (4096 * rate)) - 1
        return max(3, min(255, int(round(self.osc / (4096.0 * hz)) - 1)))

    def us_per_count(self, hz_actual):
        return 1_000_000.0 / (hz_actual * 4096.0)

    def actual_hz(self, prescale):
        return self.osc / (4096.0 * (prescale + 1))

    def set_frequency(self, hz):
        """PRE_SCALE is ONLY writable while SLEEP=1. Skipping the sleep step is the
        classic PCA9685 bug: the write appears to succeed and nothing changes."""
        pre = self.prescale_for(hz)
        old = self.rd(MODE1)
        self.wr(MODE1, (old & ~M1_RESTART) | M1_SLEEP)   # sleep, clear RESTART
        self.wr(PRE_SCALE, pre)
        self.wr(MODE1, old & ~M1_SLEEP)                  # wake
        time.sleep(0.001)                                # >=500us oscillator settle
        self.wr(MODE1, (old & ~M1_SLEEP) | M1_RESTART)   # restart the channels
        back = self.rd(PRE_SCALE)
        if back != pre:
            raise SystemExit(f"PRE_SCALE readback {back:#04x} != {pre:#04x} "
                             "-- was the SLEEP bit actually set?")
        return pre, self.actual_hz(pre)

    def init(self, hz):
        self.wr(MODE1, M1_AI | M1_ALLCALL)   # auto-increment on: needed for block writes
        # OUTDRV=1 totem-pole, OUTNE=00 -> when OE goes high the outputs are driven
        # LOW, so servos see no pulse and go limp. High-Z would let a long lead float.
        self.wr(MODE2, M2_OUTDRV)
        pre, actual = self.set_frequency(hz)
        self.all_off()
        return pre, actual

    def set_counts(self, ch, on, off):
        base = LED0_ON_L + 4 * ch
        self.b.write_i2c_block_data(self.a, base,
            [on & 0xFF, (on >> 8) & 0x0F, off & 0xFF, (off >> 8) & 0x0F])

    def set_us(self, ch, us, hz_actual):
        """Phase-stagger the ON edge per channel. With every channel starting its
        pulse on the same edge the supply sees a simultaneous surge 16x per period."""
        upc = self.us_per_count(hz_actual)
        counts = int(round(us / upc))
        if counts >= 4096:
            raise SystemExit(f"{us} us exceeds the {1e6/hz_actual:.0f} us period")
        on = (ch * 4096 // 16) % 4096
        off = (on + counts) % 4096
        self.set_counts(ch, on, off)
        return counts, on, off

    def channel_off(self, ch):
        self.set_counts(ch, 0, FULL_OFF << 8)

    def all_off(self):
        self.wr(ALL_LED_ON_L, 0); self.wr(ALL_LED_ON_L + 1, 0)
        self.wr(ALL_LED_OFF_H - 1, 0); self.wr(ALL_LED_OFF_H, FULL_OFF)

    def check_alive(self):
        """A brown-out reset is SILENT: MODE1 returns to 0x11 with SLEEP set, every
        output stops, and nothing is reported. Poll it."""
        m1 = self.rd(MODE1)
        if m1 & M1_SLEEP:
            raise SystemExit(f"MODE1={m1:#04x} has SLEEP set -- the chip reset itself "
                             "(brown-out?). Outputs are dead. Check the servo supply.")
        return m1


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bus", type=int, default=1)
    p.add_argument("--addr", type=lambda s: int(s, 0), default=0x40)
    p.add_argument("--osc", type=float, default=25_000_000,
                   help="oscillator Hz. NOMINAL 25e6; real parts run 23-27e6, so every "
                        "pulse width is off by that error until calibrated on a scope.")
    p.add_argument("--min-us", type=int, default=1000)
    p.add_argument("--max-us", type=int, default=2000)
    p.add_argument("--allow-full", action="store_true",
                   help="widen the clamp to 500..2500. Know your end stops first.")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("scan")
    q = sub.add_parser("init");  q.add_argument("--freq", type=float, default=50.0)
    q = sub.add_parser("us");    q.add_argument("ch", type=int); q.add_argument("us", type=int)
    q.add_argument("--freq", type=float, default=50.0)
    q = sub.add_parser("sweep"); q.add_argument("ch", type=int)
    q.add_argument("--from", dest="lo", type=int, required=True)
    q.add_argument("--to", dest="hi", type=int, required=True)
    q.add_argument("--step", type=int, default=20)
    q.add_argument("--dwell", type=int, default=40, help="ms per step")
    q.add_argument("--cycles", type=int, default=1)
    q.add_argument("--freq", type=float, default=50.0)
    q = sub.add_parser("off");   q.add_argument("ch", type=int)
    sub.add_parser("stop")
    a = p.parse_args()

    lo_clamp, hi_clamp = (500, 2500) if a.allow_full else (a.min_us, a.max_us)

    if a.cmd == "stop":
        # All-call: silences every channel on every board on the bus in one write.
        d = Pca9685(a.bus, 0x70, a.osc); d.all_off()
        print("all-call 0x70: every channel full-off"); return

    d = Pca9685(a.bus, a.addr, a.osc)

    if a.cmd == "scan":
        regs = d.identify()
        print(f"0x{a.addr:02x}  " + "  ".join(f"{k}={v:#04x}" for k, v in regs.items()))
        pre = regs["PRE_SCALE"]
        hz = d.actual_hz(pre)
        print(f"      prescale {pre} -> {hz:.2f} Hz, period {1e6/hz:.0f} us, "
              f"LSB {d.us_per_count(hz):.3f} us")
        print(f"      MODE1 SLEEP={'SET (outputs dead)' if regs['MODE1'] & M1_SLEEP else 'clear'}"
              f"  AI={'on' if regs['MODE1'] & M1_AI else 'OFF (block writes will not work)'}")
        return

    if a.cmd == "init":
        pre, actual = d.init(a.freq)
        print(f"init: prescale={pre} requested={a.freq} Hz actual={actual:.2f} Hz "
              f"LSB={d.us_per_count(actual):.3f} us, all channels full-off")
        return

    if a.cmd == "off":
        d.channel_off(a.ch); print(f"ch{a.ch}: full-off (no pulse, servo limp)"); return

    hz = d.actual_hz(d.rd(PRE_SCALE))
    d.check_alive()

    if a.cmd == "us":
        us = max(lo_clamp, min(hi_clamp, a.us))
        if us != a.us:
            print(f"  CLAMPED {a.us} -> {us} us (limits {lo_clamp}..{hi_clamp})")
        c, on, off = d.set_us(a.ch, us, hz)
        print(f"ch{a.ch}: {us} us -> {c} counts (on={on} off={off}) at {hz:.2f} Hz")
        return

    if a.cmd == "sweep":
        lo, hi = max(lo_clamp, min(a.lo, a.hi)), min(hi_clamp, max(a.lo, a.hi))
        print(f"ch{a.ch}: sweep {lo}..{hi} us step {a.step} dwell {a.dwell} ms "
              f"x{a.cycles} at {hz:.2f} Hz   (ctrl-c to stop)")
        try:
            for _ in range(a.cycles):
                for us in list(range(lo, hi + 1, a.step)) + list(range(hi, lo - 1, -a.step)):
                    d.set_us(a.ch, us, hz)
                    time.sleep(a.dwell / 1000.0)
                d.check_alive()
        except KeyboardInterrupt:
            print("\n  interrupted")
        finally:
            d.channel_off(a.ch)
            print(f"  ch{a.ch} left full-off (limp)")
        return


if __name__ == "__main__":
    try:
        main()
    except OSError as e:
        raise SystemExit(f"I2C error: {e}\n"
                         "  bus present? `ls /dev/i2c-1`  chip present? `i2cdetect -y 1`")
