#!/usr/bin/env bash
# Hexbot Pi bring-up -- enable and verify the hardware buses on a Raspberry Pi 5.
#
#   * 40-pin I2C at 400 kHz  -> PCA9685 servo driver boards
#   * header UART            -> the RP2040 reflex microcontroller
#
# Idempotent: safe to re-run, satisfied steps are skipped, and a re-run after a
# successful bring-up is a clean no-op that exits 0.
#
#   ./bringup-pi-hexapod.sh              configure, then stop at the reboot gate
#   ./bringup-pi-hexapod.sh --reboot     configure, reboot, wait, verify
#   ./bringup-pi-hexapod.sh --verify     verify only, change nothing
#
# Runs from a workstation and drives the Pi over SSH. Needs passwordless sudo on
# the Pi. Override the target with PI_HOST / PI_USER.
#
# The reasoning behind every config line, and the four traps that cost us a day,
# are written up in docs/pi-bringup.md. Read that before changing anything here.
set -euo pipefail

PI_HOST="${PI_HOST:-pi-hexapod.local}"
PI_USER="${PI_USER:-ben}"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=5 "${PI_USER}@${PI_HOST}")

LOG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/bringup-pi-hexapod-$(date -u +%Y%m%dT%H%M%SZ).log"
exec > >(tee -a "$LOG_FILE") 2>&1

B=$'\033[1m'; G=$'\033[32m'; Y=$'\033[33m'; R=$'\033[31m'; N=$'\033[0m'
log()  { printf '%s==>%s %s\n' "$B" "$N" "$*"; }
ok()   { printf '  %s[ ok ]%s %s\n' "$G" "$N" "$*"; }
warn() { printf '  %s[warn]%s %s\n' "$Y" "$N" "$*"; }
err()  { printf '  %s[FAIL]%s %s\n' "$R" "$N" "$*"; }
die()  { err "$*"; exit 1; }

DO_REBOOT=0; VERIFY_ONLY=0
for a in "$@"; do case "$a" in
  --reboot) DO_REBOOT=1 ;;
  --verify) VERIFY_ONLY=1 ;;
  -h|--help) sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
  *) die "unknown argument: $a" ;;
esac; done

# i2c-tools install into /usr/sbin, which is NOT on a normal user's PATH on
# Raspberry Pi OS. Without this every remote i2cdetect/i2cget call reports
# "command not found" and the script concludes i2c-tools is missing.
remote() { "${SSH[@]}" "export PATH=\$PATH:/usr/sbin; $*"; }

CFG=/boot/firmware/config.txt
MARKER='# --- Hexbot hardware (bringup-pi-hexapod.sh) ---'

log "Target: ${PI_USER}@${PI_HOST}   log: ${LOG_FILE#"$PWD/"}"
remote 'true' || die "cannot ssh to ${PI_HOST}"
ok "ssh reachable"
remote 'sudo -n true' || die "passwordless sudo not available on ${PI_HOST}"
ok "passwordless sudo"
MODEL="$(remote 'tr -d "\0" < /proc/device-tree/model')"
ok "$MODEL"
case "$MODEL" in *"Pi 5"*) ;; *) warn "not a Pi 5 -- the pin functions and bus numbering below are Pi 5 specific" ;; esac

# ---------------------------------------------------------------- phase 1: before
log "Phase 1 -- before-state (phase 4 diffs against this)"
remote 'ls -l /dev/i2c-* 2>&1 | sed "s/^/    /" || true'
remote 'echo "    /sys/class/pwm: $(ls /sys/class/pwm 2>/dev/null | tr "\n" " ")"'
remote 'pinctrl get 2,3 2>&1   | sed "s/^/    /" || true'
remote 'pinctrl get 14,15 2>&1 | sed "s/^/    /" || true'
remote 'echo "    $(vcgencmd measure_temp) $(vcgencmd get_throttled) clock=$(vcgencmd measure_clock arm | cut -d= -f2)"'
remote 'echo "    loadavg: $(cut -d" " -f1-3 /proc/loadavg)"'

# ------------------------------------------------------------- phase 2: config.txt
if [[ $VERIFY_ONLY -eq 0 ]]; then
  log "Phase 2 -- $CFG"
  if remote "grep -qF '$MARKER' $CFG"; then
    ok "Hexbot block already present -- no change"
    NEEDS_REBOOT=0
  else
    BAK="${CFG}.bak-$(date -u +%Y%m%dT%H%M%SZ)"
    remote "sudo cp -n $CFG $BAK"
    ok "backed up to $BAK"
    remote "sudo tee -a $CFG >/dev/null" <<'EOF'

# --- Hexbot hardware (bringup-pi-hexapod.sh) ---
# 40-pin I2C, for the PCA9685 servo driver boards.
dtparam=i2c_arm=on
# 400 kHz is a PREREQUISITE, not an optimisation. Setting all 16 channels of one
# PCA9685 is a 64-byte auto-increment block write from LED0_ON_L; with the address
# and register byte that is ~66 bytes, ~594 bit-times including ACKs. At the
# default 100 kHz that is 5.9 ms per board -- 11.9 ms for two boards, which is 60%
# of a 20 ms control frame. At 400 kHz it is 1.5 ms per board. The PCA9685 itself
# is rated for 1 MHz Fast-mode Plus, so 400 kHz is conservative.
dtparam=i2c_arm_baudrate=400000
# Header UART on GPIO14/15 -> /dev/ttyAMA0, for the RP2040 reflex link.
dtparam=uart0=on
# dtoverlay=pwm-2chan
#   NOT enabled, for two independent reasons:
#   1. Actuation goes over I2C to the PCA9685, so zero Pi-native PWM channels are
#      needed. The Pi only has 4 of them and this robot has 24 joints.
#   2. It CONFLICTS with dtparam=audio=on, which Raspberry Pi OS sets by default.
#      Per /boot/firmware/overlays/README, "the onboard analogue audio output uses
#      both PWM channels." Enabling both silently breaks one of them, with no
#      error reported anywhere. Drop audio=on first if you ever need Pi-native PWM.
# --- end Hexbot ---
EOF
    ok "appended the Hexbot block"
    NEEDS_REBOOT=1
  fi

  # ---- i2c-dev ----
  # dtparam=i2c_arm=on registers the ADAPTER but does NOT create the /dev/i2c-1
  # CHARACTER DEVICE. That is the i2c-dev module's job, and it is not autoloaded.
  # Failure mode: /sys/bus/i2c/devices/i2c-1 exists, pinctrl shows SDA1/SCL1, and
  # yet `i2cdetect -l` prints nothing and /dev/i2c-1 is absent -- which looks
  # exactly like a hardware fault. raspi-config does both steps, which is why
  # "enable I2C in raspi-config" works and a hand-written dtparam appears not to.
  log "Phase 2b -- i2c-dev kernel module"
  remote 'lsmod | grep -q "^i2c_dev"' || { remote 'sudo modprobe i2c-dev' && ok "i2c-dev loaded"; }
  remote 'test -e /etc/modules-load.d/i2c-dev.conf' \
    || { remote 'echo i2c-dev | sudo tee /etc/modules-load.d/i2c-dev.conf >/dev/null' \
         && ok "i2c-dev persisted to /etc/modules-load.d/i2c-dev.conf"; }
  remote 'lsmod | grep -q "^i2c_dev"' && ok "i2c-dev present" || warn "i2c-dev still not loaded"
else
  NEEDS_REBOOT=0
fi

# ---------------------------------------------------------------- phase 3: reboot
if [[ ${NEEDS_REBOOT:-0} -eq 1 ]]; then
  if [[ $DO_REBOOT -eq 1 ]]; then
    log "Phase 3 -- rebooting"
    remote 'sudo systemd-run --on-active=1 --timer-property=AccuracySec=100ms systemctl reboot' >/dev/null 2>&1 \
      || remote 'sudo nohup sh -c "sleep 1; systemctl reboot" >/dev/null 2>&1 &' || true
    sleep 10
    for i in $(seq 1 30); do
      remote 'true' 2>/dev/null && { ok "back up after ~$((10 + i*5))s"; break; }
      sleep 5
      [[ $i -eq 30 ]] && die "did not come back within 160s"
    done
  else
    warn "A REBOOT IS REQUIRED for i2c_arm and uart0 to take effect."
    warn "Re-run with --reboot, or reboot by hand and re-run with --verify."
    exit 0
  fi
fi

# ---------------------------------------------------------------- phase 4: verify
log "Phase 4 -- verification"
FAILED=0
chk() { # chk <label> <remote test> <expectation>
  if remote "$2" >/dev/null 2>&1; then ok "$1"; else err "$1 -- expected $3"; FAILED=1; fi
}
chk "/dev/i2c-1 exists"       'test -e /dev/i2c-1'                 "i2c_arm=on, i2c-dev loaded, and a reboot"
# On a Pi 5 the header-I2C alt function is a3 (SDA1/SCL1). On earlier models it
# was a1, so a check that greps for "a1" FAILS on a correctly configured Pi 5.
chk "GPIO2/3 are I2C"         'pinctrl get 2,3 | grep -q "SDA1"'   "a3 SDA1/SCL1, not none"
chk "/dev/ttyAMA0 exists"     'test -e /dev/ttyAMA0'               "uart0=on and a reboot"
chk "GPIO14/15 are UART"      'pinctrl get 14,15 | grep -q "a4"'   "a4 TXD0/RXD0"
# xxd is NOT installed on Raspberry Pi OS -- use od. And the clock lives on the
# RP1 device-tree node, not under /sys/bus/i2c/devices/i2c-1/of_node, which has
# no clock-frequency property at all.
chk "I2C bus clock is 400 kHz" \
  'test "$(od -An -tu4 --endian=big /proc/device-tree/axi/pcie@1000120000/rp1/i2c@74000/clock-frequency 2>/dev/null | tr -d " ")" = 400000' \
  "400000"
remote 'pinctrl get 2,3; pinctrl get 14,15' 2>&1 | sed 's/^/    /' || true

# NOTE: /dev/ttyAMA10 (symlinked /dev/serial0) exists on a stock Pi 5 and is the
# DEBUG UART on the 3-pin connector -- NOT the 40-pin header. Always confirm the
# header UART with pinctrl, never by the presence of a device node.

# ------------------------------------------------------- phase 5: I2C bus contents
log "Phase 5 -- I2C bus"
if remote 'test -e /dev/i2c-1'; then
  remote 'i2cdetect -y -r 1' 2>&1 | sed 's/^/    /'
  ADDRS=$(remote 'i2cdetect -y -r 1' 2>/dev/null | tail -n +2 | cut -d: -f2 \
          | tr -s " " "\n" | grep -E '^[0-9a-f]{2}$' | tr '\n' ' ')
  ok "responding addresses: ${ADDRS:-none}"
  # PCA9685 address = a base plus the A0..A5 jumpers. 0x40 is the base for servo
  # driver boards; 0x60 is the base for Adafruit's DC+Stepper Motor HAT, whose
  # channels feed H-bridges out to screw terminals and CANNOT pulse a servo.
  FOUND=""
  for a in $ADDRS; do case "$a" in
    4[0-9a-f]) FOUND="$FOUND 0x$a"; ok "0x$a -- PCA9685 servo driver (base 0x40 + jumpers)" ;;
    6[0-9a-f]) warn "0x$a -- DC+Stepper Motor HAT family (base 0x60 + jumpers)." ;
               warn "       Its channels drive H-bridges, not servo headers. Cannot pulse servos." ;;
    70) ok "0x70 -- PCA9685 all-call. One 3-byte write here stops every board at once." ;;
  esac; done
  if [[ -n "$FOUND" ]]; then
    for h in $FOUND; do
      # Three registers at their documented power-on defaults is positive chip
      # identification. An address merely ACKing proves only that something is there.
      M1=$(remote "i2cget -y 1 $h 0x00" 2>/dev/null || echo "?")
      M2=$(remote "i2cget -y 1 $h 0x01" 2>/dev/null || echo "?")
      PS=$(remote "i2cget -y 1 $h 0xFE" 2>/dev/null || echo "?")
      echo "    $h  MODE1=$M1 (expect 0x11)  MODE2=$M2 (expect 0x04)  PRE_SCALE=$PS (expect 0x1e, ~197 Hz)"
      [[ "$M1" == "0x11" ]] && ok "$h MODE1 at power-on default -- a fresh PCA9685" \
        || warn "$h MODE1 is not 0x11 -- something has already configured this chip. Find out what."
    done
  else
    warn "no PCA9685 found in 0x4x. Servo work needs at least one 0x40-family board;"
    warn "24 joints needs two, at 0x40 and 0x41 (bridge A0 on the second)."
  fi
else
  err "cannot probe: /dev/i2c-1 missing"; FAILED=1
fi

# --------------------------------------------------------------- phase 6: thermal
log "Phase 6 -- thermal"
# ONE sample, reused. Sampling twice prints a temperature the verdict did not use.
# `read` returns non-zero at EOF when the input has no trailing newline, which
# under `set -e` kills the script here with no output at all. Hence the `|| true`
# AND the \n -- either alone is enough, both is cheap insurance.
read -r T TH CLK < <(remote 'printf "%s %s %s\n" "$(vcgencmd measure_temp | sed "s/temp=//;s/.C//")" "$(vcgencmd get_throttled | cut -d= -f2)" "$(vcgencmd measure_clock arm | cut -d= -f2)"') || true
[[ -n "${T:-}" && -n "${TH:-}" ]] || { warn "could not read thermal state"; T=0; TH=0x0; CLK=?; }
echo "    temp=${T}C throttled=${TH} clock=${CLK}"
remote 'echo "    governor: $(cat /sys/devices/system/cpu/cpufreq/policy0/scaling_governor)   loadavg: $(cut -d" " -f1-3 /proc/loadavg)"'
# get_throttled bits: 0 under-voltage now, 1 arm-freq capped now, 2 throttled now,
# 3 soft temp limit now; bits 16-19 are the same four "has occurred since boot".
# An under-voltage bit points at the PSU; a thermal bit points at cooling or load.
if (( $(printf '%d' "$TH") & 0x1 )); then
  warn "UNDER-VOLTAGE now (throttled=$TH) -- suspect the power supply, not cooling"
fi
if (( $(printf '%d' "$TH") & 0x6 )); then
  warn "actively throttling (throttled=$TH at ${T}C) -- fix cooling before trusting any timing"
elif awk "BEGIN{exit !($T < 60)}"; then
  ok "${T}C, nothing throttling"
else
  warn "${T}C -- not throttling, but warm. Re-check under control-loop load."
fi
RPM="$(remote 'cat /sys/devices/platform/cooling_fan/hwmon/*/fan1_input 2>/dev/null' 2>/dev/null || true)"
if [[ -n "$RPM" ]]; then
  ok "fan tachometer: ${RPM} rpm"
else
  warn "no fan tachometer -- either no fan fitted, or it is on a header with no tach line."
  warn "     'fan rpm = 0' is a telemetry channel worth having on a robot that holds servos."
fi

# ----------------------------------------------------- phase 7: control-plane (opt)
log "Phase 7 -- Krill services (optional; skipped if not installed)"
for u in krill krill-pi4j; do
  if remote "systemctl list-unit-files ${u}.service >/dev/null 2>&1"; then
    printf '    %-12s active=%-8s enabled=%s\n' "$u" \
      "$(remote "systemctl is-active $u" || true)" "$(remote "systemctl is-enabled $u" || true)"
  else
    printf '    %-12s not installed\n' "$u"
  fi
done
warn "krill-pi4j cannot reach hardware yet: it never selects a Pi4J provider, so PWM,"
warn "I2C, GPIO and SPI all return success and do nothing. See krill-oss#244."

if [[ $FAILED -eq 0 ]]; then log "${G}BRING-UP OK${N}"; else die "one or more checks FAILED (see above)"; fi
