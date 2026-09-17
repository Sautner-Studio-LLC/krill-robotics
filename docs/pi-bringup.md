# Raspberry Pi 5 bring-up for I²C servo control

Everything here is Pi 5 (RP1) specific and was measured on Raspberry Pi OS trixie,
kernel 6.18. `scripts/bringup-pi-hexapod.sh` automates all of it; this document is
the *why*, and the four traps that each cost real time.

## What gets enabled, and why

```
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=400000
dtparam=uart0=on
```

### 400 kHz is a prerequisite, not an optimisation

Setting all 16 channels of a PCA9685 is a 64-byte auto-increment block write starting
at `LED0_ON_L` (0x06). With the address and register byte that is ~66 bytes, or about
594 bit-times once ACKs are counted.

| bus speed | one board | two boards | share of a 20 ms frame |
|---|---|---|---|
| 100 kHz (default) | 5.9 ms | **11.9 ms** | **60 %** |
| 400 kHz | 1.5 ms | 3.0 ms | 15 % |

A 50 Hz control loop cannot give up 60 % of its frame to the bus. The PCA9685 is rated
for 1 MHz Fast-mode Plus, so 400 kHz is the conservative choice, not an aggressive one.

### Why `pwm-2chan` is deliberately *not* enabled

Two independent reasons:

1. Actuation goes over I²C to the PCA9685, so **zero** Pi-native PWM channels are
   needed. The Pi only has four, and this robot has 24 joints.
2. **It conflicts with `dtparam=audio=on`**, which Raspberry Pi OS sets by default.
   `/boot/firmware/overlays/README` states plainly that "the onboard analogue audio
   output uses both PWM channels." Enable both and one of them silently stops working,
   with no error reported anywhere. Drop `audio=on` first if you ever need Pi-native PWM.

## ⚠ Trap 1 — `dtparam=i2c_arm=on` does not create `/dev/i2c-1`

This is the one that looks most like a hardware fault.

`dtparam=i2c_arm=on` registers the **adapter**. After a reboot you get:

- `/sys/bus/i2c/devices/i2c-1`, named *Synopsys DesignWare I2C adapter*
- `pinctrl get 2,3` showing `GPIO2 = SDA1`, `GPIO3 = SCL1`

…and yet **`/dev/i2c-1` does not exist** and `i2cdetect -l` prints nothing at all.

The character device is created by the **`i2c-dev`** kernel module, which is not
autoloaded. `raspi-config` loads it *and* sets the dtparam, which is exactly why
"enable I²C in raspi-config" works and a hand-written `config.txt` line appears not to.

```bash
sudo modprobe i2c-dev
echo i2c-dev | sudo tee /etc/modules-load.d/i2c-dev.conf   # persist across reboots
```

## ⚠ Trap 2 — on a Pi 5 the header-I²C alt function is `a3`, not `a1`

```
 2: a3    pu | hi // GPIO2 = SDA1
 3: a3    pu | hi // GPIO3 = SCL1
```

On earlier models it was `a1`. A verification script that greps for `a1` therefore
**fails on a correctly configured Pi 5**. Grep for `SDA1`/`SCL1` instead — the symbolic
name is stable across models and the alt-function number is not.

## ⚠ Trap 3 — `/dev/ttyAMA10` is not the header UART

A stock Pi 5 already has `/dev/ttyAMA10`, symlinked as `/dev/serial0`, and a user in
the `dialout` group can open it. It is the **debug UART on the 3-pin connector**, not
the 40-pin header.

`dtparam=uart0=on` gives you `/dev/ttyAMA0` on GPIO14/15. **Confirm it with `pinctrl`,
never by the presence of a device node:**

```
14: a4    pn | hi // GPIO14 = TXD0
15: a4    pu | lo // GPIO15 = RXD0
```

## ⚠ Trap 4 — tooling that is not where you expect

- **`i2c-tools` installs into `/usr/sbin`**, which is not on a normal user's `PATH`.
  Every `i2cdetect`/`i2cget` call over SSH reports *command not found*, and a script
  concludes the package is missing when it is installed. `export PATH=$PATH:/usr/sbin`.
- **`xxd` is not installed.** Use `od -An -tu4 --endian=big`.
- The I²C **`clock-frequency`** property lives on the RP1 device-tree node,
  `/proc/device-tree/axi/pcie@1000120000/rp1/i2c@74000/`. It is *not* under
  `/sys/bus/i2c/devices/i2c-1/of_node/`, which has no such property, so a check there
  reads empty and looks like the baudrate never applied.

## Identifying what is on the bus

An address that ACKs proves only that *something* is there. Three registers at their
documented power-on defaults is positive identification of a PCA9685:

```bash
i2cget -y 1 0x40 0x00   # MODE1     -> 0x11  (SLEEP | ALLCALL) on a fresh chip
i2cget -y 1 0x40 0x01   # MODE2     -> 0x04  (OUTDRV, totem-pole)
i2cget -y 1 0x40 0xFE   # PRE_SCALE -> 0x1e  (=30, ~197 Hz factory default)
```

If `MODE1` is anything other than `0x11`, something has already configured the chip.
Find out what before writing to it.

### Address bases matter more than the label on the board

A PCA9685's address is a **base plus the A0–A5 solder jumpers**:

| base | family | can it pulse a servo? |
|---|---|---|
| **0x40** | servo / PWM driver boards | **yes** — 3-pin GND/V+/SIG headers |
| **0x60** | Adafruit DC+Stepper Motor HAT | **no** — channels feed H-bridges to screw terminals |
| 0x70 | all-call (both, by default) | n/a — see below |

So `0x61` is `0x60 + A0`: a motor HAT with one jumper bridged. Both families carry the
same PCA9685 silicon, which is why a register-level identity check passes on either and
tells you nothing about whether a servo can physically connect.

**Trust the bus address over the silkscreen, the datasheet you think applies, or the
HAT ID EEPROM.** We had a board whose EEPROM reported a DC+Stepper Motor HAT while it
was verbally described as a servo HAT; the bus answered `0x61` and settled it in one
command. The physical tell is unambiguous once you know to look: **screw terminals mean
motor driver, four rows of 3-pin headers mean servo driver.**

### The all-call address is a feature — keep it

Every PCA9685 answers `0x70` by default (`ALLCALLADR` = 0xE0). That makes

```bash
i2cset -y 1 0x70 0xFD 0x10    # ALL_LED_OFF_H, full-off bit
```

a **software emergency stop that silences every channel on every board in one 3-byte
write** — about 60 µs at 400 kHz. Worth keeping as the software-tier complement to a
hardware `OE` line.

## Thermal

`vcgencmd get_throttled` is a bitfield and the two halves mean different things:

| bit | meaning |
|---|---|
| 0 | under-voltage **now** |
| 1 | ARM frequency capped **now** |
| 2 | throttled **now** |
| 3 | soft temperature limit active **now** |
| 16–19 | the same four, "has occurred since boot" |

Read the distinction carefully before diagnosing. `0xe0000` is *history only* — nothing
is wrong at that instant. `0xe0006` is actively throttling. And **an under-voltage bit
points at the power supply while a thermal bit points at cooling or load** — they are
not interchangeable, and chasing the wrong one wastes an afternoon.

Worth knowing: a runaway process is far more likely than inadequate cooling. We measured
85.6 °C and active throttling on a Pi 5 that dropped to **57.6 °C with `get_throttled=0x0`
and a steady 2.4 GHz** once a single misbehaving service was stopped. **Diagnose load
before buying a fan.**
