# krill-robotics

A radial six-legged robot with a Raspberry Pi 5 brain, built around
[Krill](https://krill.zone) as its control system.

The robot itself is not smart. It is a peripheral with a nervous system: reflexes in
hardware, motion on the Pi, and judgement somewhere else on the mesh. This repo holds the
Pi-side and motion-side software, plus the operational notes for getting a Pi 5 to drive
24 servos over I²C.

> **Brand new project, started September 2026 — stay tuned.**
>
> One leg exists and moves: four joints under coordinated command, driven from a Pi 5 over
> I²C. What is published here so far is the Pi bring-up automation, the bench tooling, and
> the design rules below — the things that turned out to be worth writing down. The motion
> library is next, and it is waiting on measured link lengths rather than on code.

## Shape of the thing

Six legs at 60°, **no front and no back**, four degrees of freedom per leg — coxa yaw,
hip lift, knee, ankle — for 24 joints total. Actuation is two PCA9685 boards over I²C.

Three tiers, and the line between them is the design:

| tier | where | latency | job |
|---|---|---|---|
| reflex | RP2040 microcontroller | 1–10 ms | foot contact, tilt limit, obstacle stop. Kills servo outputs **in hardware**. |
| motion | Raspberry Pi 5 | 20 ms (50 Hz) | inverse kinematics, gait, body pose, sensor fusion |
| cognition | elsewhere on the Krill mesh | ~1 s | vision, mission intent, strategy |

**Krill is the nervous system and the safety-policy layer, not the servo driver.** 24
servos at 50 Hz is not 24 graph nodes — the robot is one node with a small surface of
intent, telemetry and policy. The motion code carries no Krill dependency at all, so it
is unit-testable with no hardware and no server.

## Two design rules worth knowing before reading anything else

**The robot never stands still.** When it stops it drops onto its belly and the servos go
cold. So there is no standing-hold state, thermal steady state is never reached, and a
belly-down pose that is stable *with torque off* is the central invariant of the machine —
it is the rest state, the charge state and the failure state at once.

**The fourth joint exists for reach, not torque.** The femur is much shorter than the
tibia, so the leg is nearly a fixed-length strut on a ball joint and its reachable shell is
thin. Adding the ankle takes the usable body-height travel from 125 mm to 295 mm. A stair
riser is 178 mm — so a three-jointed version of this robot physically cannot lift its body
one step.

## Layout

```
scripts/    operational automation (idempotent, re-runnable, logs what it does)
docs/       design rules and hard-won operational notes
logs/       script output (gitignored)
```

## Getting a Pi 5 ready

```bash
scripts/bringup-pi-hexapod.sh --reboot     # configure, reboot, verify
scripts/bringup-pi-hexapod.sh --verify     # verify only, change nothing
```

Enables the 40-pin I²C bus at 400 kHz and the header UART, then verifies both and
identifies whatever is on the bus. Idempotent — a re-run after success is a clean no-op.
Override the target with `PI_HOST` / `PI_USER`.

**[docs/pi-bringup.md](docs/pi-bringup.md)** is the interesting half: why 400 kHz is a
prerequisite rather than a tuning knob, and the four traps that each cost real time —
including that `dtparam=i2c_arm=on` does **not** create `/dev/i2c-1`, which looks exactly
like a hardware fault.

**[docs/hardware.md](docs/hardware.md)** has the as-built geometry, the static torque model
and the contact modes.

## Contributing

Early days and the interfaces are still moving, but bug reports and notes from anyone
running a PCA9685 off a Pi 5 are very welcome — particularly measurements, since several
numbers in `docs/` are still estimates awaiting a caliper.

## Licence

MIT. See [LICENSE](LICENSE).
