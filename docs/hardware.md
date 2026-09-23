# Hexbot — hardware and design rules

The machine as actually built, and the handful of rules that constrain everything else.
Numbers marked **estimate** are awaiting measurement; the discipline is that every
dimension lives in exactly one place in code and declares its provenance.

## Layout

Radial — six legs at 60°, **no front and no back**. The consequences of that choice show
up everywhere: rotation in place is a single joint per leg, there is no turning circle,
and "which way is forward" is a software decision rather than a mechanical one.

## Per leg: 4 DOF, 24 servos total

| # | joint | axis | carries gravity torque | servo class |
|---|---|---|---|---|
| 1 | coxa yaw | **vertical** | no | DS5180SG, 98 kg·cm |
| 2 | hip lift | horizontal | yes | DS5180SG, 98 kg·cm |
| 3 | femur joint | horizontal, ∥ to 2 | yes | DS5180SG, 98 kg·cm |
| 4 | tibia joint | horizontal, ∥ to 2 and 3 | yes | DS3225MG, 25 kg·cm |

Joints 2–4 are three **coplanar pitch joints** in one vertical plane, and joint 1 rotates
that plane about the vertical. So the leg is a **3-link planar chain with one redundant
DOF**, not the 2-link law-of-cosines problem a 3-DOF hexapod leg gives you.

> **The coxa's vertical axis carries no gravity torque but is not over-specced.** The
> moment of a vertical force about a vertical axis is identically zero, so it holds
> nothing up — but it carries the *entire* horizontal reaction: all propulsion, all
> turning, and every side load. It is the joint that resists the robot being shoved.

### Link lengths

| | value | status |
|---|---|---|
| hip circle radius | 85–95 mm | design target |
| hip lift axis → knee | ~60 mm | **estimate — highest-leverage unknown** |
| knee → ankle | **350 mm** | measured |
| ankle → foot contact | ~80 mm | **estimate** |
| total mass | ~7 kg | **estimate** |

Mass bottom-up: 24 servos are 6 × (2 × 0.17 + 2 × 0.07) = **2.88 kg of servos alone**,
plus roughly 1.9 kg of printed leg structure and 1.5–2.5 kg of chassis, pack and
electronics. Mass is the most leveraged unmeasured quantity in the machine — every
workspace limit scales as 1/M.

## Why the fourth joint exists

Not for torque. **For reach range.** Without it the leg is one short link plus one long
rigid strut, so the reachable shell is a thin annulus — the foot can swing, but it cannot
move much toward or away from the hip.

Recomputed on the **leg v2** measured lengths (112 / 330 / 235 mm), comparing against a
3-DOF leg with the tibia joint welded straight:

| | hip→contact shell | body-height travel at 100 mm stance offset |
|---|---|---|
| 3 DOF (tibia welded) | [453, 677] mm — 224 mm deep | 228 mm |
| 4 DOF | [0, 677] mm — **677 mm deep** | **670 mm** |

The min reach collapses to zero because the longest link (330) is shorter than the other two
together (347) — the leg can fold back on itself. A US residential stair riser is 178 mm, and
a 3-DOF leg clears that only near one stance offset, with no margin either side. The fourth
joint is what turns a thin shell into a solid one, and that is its justification.

## The torque model

```
tau_joint = F * (horizontal distance from that joint's axis to the CONTACT POINT) * SF
F         = M * g * loadShareWorstCase
```

**The structural result:** for a vertical load, `tau_hip = F · u_c`, where `u_c` is the
horizontal offset from the hip-lift axis to the contact point. It contains **no joint
angle at all** — sweeping the leg's redundant DOF at a fixed foot target leaves hip torque
invariant to the last digit. So the redundancy cannot reduce hip torque; it only trades
between knee and ankle. That splits the guard into two separate mechanisms: a **hard
workspace limit** on contact placement, and **posture optimisation** inside it.

A useful closed form falls out for the ankle: `tau_ankle = F · L4 · cos φ`, where φ is the
absolute angle of the foot segment. It is **maximal at flat-foot** and zero at φ = ±90°.

> ⚠ **Keep ankle→foot short.** Because ankle torque peaks in the flat-footed pose — the
> one you want for traction — a long foot segment makes flat-footed standing torque-
> infeasible and forces a toe-down or heel-down attitude instead. Under about 100 mm.

### Two parameters, not one

`loadShareWorstCase` (physics) and `safetyFactor` (design margin) must be separate.
Tripod load does not split evenly by `1/n`; it splits by where the centre of mass falls
in the support triangle, reaching 1.0 at a vertex. Folding that uncertainty into the
safety factor means a nominal 2× margin is really about 1.33×, and you cannot see which
one you are spending.

### Derate against measured pack voltage

Servo torque falls with pack voltage, so a fixed ceiling is wrong in both directions at
once — it wastes an 80 kg·cm joint's capability while being optimistic about a 45 kg·cm
joint near cutoff. Derate the published curve and evaluate it at the actual voltage. On a
light pack the voltage sags visibly across one mission, so this is doing real work from
the first walk, not just near cutoff.

### The term that is easy to omit

`tau_hip = c_u · R_w − c_w · R_u`, and `c_w` is negative (the contact is below the hip).
**Body height is a full moment arm on every horizontal force.** At a 341 mm body height a
mere 10 N of drive or side load contributes 3.41 N·m at the hip — larger than the entire
vertical-load budget. And it is *worse* in the tall tucked stance that a vertical-only
analysis recommends. **The guard must take a force vector, never a scalar.**

### The leg's own mass is not negligible

Roughly 0.46 kg sits outboard of each hip-lift axis. That is ~12 % of the hip budget in a
tucked stance and **~27 % for a leg extended in swing**. The 27 % decides it: a swing leg
carries no external load, so a model without self-mass computes exactly zero for the
swing-phase check and the check is vacuous.

## Contact modes

The leg can present different links to the ground, and the modes differ in **constraint
arity**, not merely in contact offset — so each needs its own solve rather than sharing
one with an offset parameter.

| mode | contact | position constraints | free DOF |
|---|---|---|---|
| `PLANTIGRADE` | foot sole or toe | 3 (point) | 1 → the foot attitude φ |
| `ANKLE_PLANT` | distal tibia | 3 (point) | 0 — exactly determined |
| `KNEE_PLANT` | knee pad | 3 (point) | 0, and only a femur's worth of reach |
| `SHIN_BRACE` | tibia *surface* | 2 point + 1 orientation | 0, and the stance offset becomes an **output** |

`SHIN_BRACE` is the one that breaks a naive abstraction: with a line contact you cannot
command a contact point at all, only a posture. Toe contact is **not** a mode — it is a
different named point on the same foot link, so model contact features (a point plus a
normal, fixed in a link frame) and digitigrade falls out for free.

`ANKLE_PLANT` earns its place three ways: the foot and ankle servo retract *above* the
contact and are protected on rubble, `tau_ankle` becomes structurally zero so that servo
can be de-energised, and it is the degraded mode if an ankle servo fails — the leg still
walks.

### Knee contact is the high-load mode

Not because folding shortens a lever — hip torque is mode-independent at a given stance
offset. It is because **only the tibia is long enough to place a contact point directly
beneath the hip axis at low body height**, and that pose is a vertical compression strut:

| femur angle | body height | contact offset | hip torque |
|---|---|---|---|
| **−90°** | **60 mm** | **0 mm** | **0.000 N·m** |
| −70° | 56 mm | 20.5 mm | 0.704 N·m |
| −60° | 52 mm | 30.0 mm | 1.030 N·m |

Plantigrade can reach zero offset too, but only at ~350 mm body height, where the support
polygon is small relative to the centre-of-mass height. With the unloaded tibiae splayed
outward as outriggers, knee contact reaches a **78.9° tipping angle** against 16.4° for a
plantigrade tripod at the same span.

## ⚠ Rule: the robot never stands still

**Whenever it stops, it drops onto its belly and the servos go cold.** There is no
standing-hold state at all, and this is the most load-bearing rule in the project:

- The **static holding-torque case is not the design case.** The binding cases are dynamic
  load transfer and body lift.
- **Thermal steady state is never reached**, so 24 large servos never cook holding a pose.
  A duty-cycle model should accumulate over *motion*, not over holding.
- **A belly-down pose that is stable with torque off is the central invariant of the
  machine** — it is simultaneously the rest state, the charge state and the failure state.
  If that pose needs power to hold, everything downstream of it fails.

### Two stops, and conflating them is the bug

| | belly drop | emergency torque off |
|---|---|---|
| what | commanded, sequenced lower onto the chassis, *then* cut torque | cut all outputs immediately |
| driven by | the motion tier, on any ordinary stop | the reflex tier, in hardware |
| result | controlled, repeatable, charge-ready | **the robot falls from wherever it was** |

The normal stop is a **motion**, not a freeze. A corollary worth stating: because the
reflex cut drops the body from whatever height it was carrying, **keeping default walking
height low is a safety property**, not a style choice.

## Mission profile

Short bursty missions of 10–20 minutes, then return to a charge plate and rest belly-down
with servos cold. Multiple plates rather than one home base, so "return to dock" means
**the nearest** plate — dock poses are a *set*, the charge reserve takes the minimum
distance, and alignment must be repeatable across plates rather than tuned to one.

This profile is why cooling is undemanding: a 20-minute duty cycle followed by a charge
break never reaches thermal steady state. It also sizes the pack — a 20-minute burst is on
the order of 25–37 Wh, so a pack sized for hour-long missions is dead weight, and weight
is the most leveraged number in the torque budget.

## Measured geometry — leg v2 (Fusion, 2026-09-23)

`cad/export/leg-geometry.json` is schema **`hexbot-leg-geometry/2`**, and it says outright that
it replaces v1: *"old leg design, pre-redesign; discard it."* **Bench numbers taken before
2026-09-23 do not transfer.**

### Naming — three words for the same two joints

The extract calls the pitch joints `knee` and `ankle`; Ben calls them the femur and tibia
joints; the channel map calls them 2 and 3. All three mean the same pair, and v1's own
parameter comments already used *femur servo* / *tibia servo*. This doc uses **femur joint
(ch2)** and **tibia joint (ch3)**, because that is what the links they drive are called.

### What changed

| | v1 | v2 | |
|---|---|---|---|
| coxa yaw → hip lift | 0.08 mm | 1.41 mm | axes still effectively intersect |
| hip lift → femur joint | 162.25 mm | **111.95 mm** | −50 mm: the servo moved in toward the hip |
| femur joint → tibia joint | 352.79 mm | **330.32 mm** | −22 mm |
| tibia joint → toe | 235.19 mm | 234.81 mm | unchanged |
| reach from the hip-lift axis | 750 mm | **677 mm** | −73 mm |
| leg mass | 643.8 g | **730.0 g** | +86 g |
| servos per leg | 2 × 165 g + 2 × 60 g | **3 × 165 g + 1 × 60 g** | the femur joint is now a DS5180SG |

**Mass:** 6 × 730 g + 253.2 g body core = **4.63 kg modelled**, excluding fasteners, wiring,
electronics, battery and bearings — call it **5.3–6.0 kg** assembled. **3.33 kg of that is
servo**: 59% of the machine, and 76% of a single leg. The structure is not the mass problem
and never was.

> The coxa-yaw and hip-lift axes still meet (1.41 mm common normal), so `coxaLateralOffsetMm = 0`
> stays literally true and the coxa closed form still simplifies for real.

### ⚠ The binding joint moved to the tibia

Putting an 80 kg servo on the femur joint did exactly what it was meant to. It also handed the
constraint to the one joint still on a DS3225MG — and that joint drives the longest link that
reaches the ground, 234.81 mm of it.

In-budget levers, DS5180SG derated to **90.6 kg·cm at 6.6 V**, DS3225MG at **25 kg·cm**:

| M | share | SF | coxa / hip / femur | **tibia** |
|---|---|---|---|---|
| 5.3 kg | ideal ⅓ | 2.0 | 256 mm | **71 mm** |
| 5.6 kg | ideal ⅓ | 2.0 | 243 mm | **67 mm** |
| 5.6 kg | worst 0.5 | 1.5 | 216 mm | **60 mm** |
| 5.6 kg | worst 0.5 | 2.0 | 162 mm | **45 mm** |
| 5.9 kg | worst 0.5 | 2.0 | 154 mm | **42 mm** |

At the honest design point — worst-case share, SF 2.0 — **the tibia link has to stay within
about 11° of vertical.** Tilt it 30° off vertical and its lever is 117 mm, which is 2.6× the
budget. `tau_tibia = F · L4 · cos(phi)` is maximal at flat-foot, so **this leg cannot stand
flat-footed**: the foot has to hang.

That is a design statement about the toe, not a software limit. Either the tibia joint gets one
of the spare 80 kg servos (the lever cap goes to 162 mm and the constraint disappears), or the
toe geometry has to guarantee the contact point sits under the tibia joint.

### Reach-rich, stance-poor

The three 80 kg joints all cap at the **same** 162 mm of horizontal offset, because each one's
lever is the horizontal distance from its own axis to the toe and the hip's is the largest. So:

- **Height is nearly free.** With the tibia vertical, the leg reaches **645–673 mm** below the
  hip axis anywhere in the usable offset range — the height barely varies with stance width.
- **Width is the scarce resource.** 162 mm of offset plus a 72.7 mm hip circle is a foot radius
  of 235 mm, **span ≈ 469 mm**.

Which makes the stability trade one-sided — spend nothing on height you do not need:

| body / COM height | tripod tipping | 5 legs down |
|---|---|---|
| 200 mm | **30.7°** | 43.9° |
| 250 mm | 25.4° | 37.6° |
| 300 mm | 21.6° | 32.6° |
| 400 mm | 16.5° | 25.7° |
| 565 mm | 11.9° | 18.8° |

Standing low is worth more than a stronger servo here. At 200 mm the tripod tipping angle is
30.7°, within sight of the 32.5° stair figure that the old geometry missed by a factor of two.

### ⚠ Two things v1 had and v2 does not

1. **No standing pose.** v1 shipped a parameter-driven standing pose (`leg_hip_lift 20°`,
   `leg_femur 60°`, `leg_tibia −75°`) with the toes on a ground plane; v2's saved pose is folded,
   toe ~45 mm below the mount seat. So *"825 mm span, body 325 mm up"* is gone and the stance
   numbers above are derived here, not read from the model.
2. **No user parameters.** The `HexBot/Params` file is not derived into this assembly, so joint
   zero and direction relative to servo pulse width are **undefined in the model**. The
   pulse ↔ angle mapping now has to come entirely from the bench.

Neither is wrong — it is a single-leg assembly mid-redesign — but both were load-bearing and
both should come back before the gait engine needs them. There is also **no six-leg assembly**,
so hip circle radius and mounting azimuth are unavailable; **72.7 mm is stale** and every span
figure above inherits that.

### Still missing from the model

- DS3225MG datasheet — its torque figure is inferred and it is the binding number again
- Joint rotation limits — none modelled on any of the four joints
- The **pulse-width ↔ model-angle mapping**, now with no parameters to anchor it
- 690–1315 g of unmodelled mass, which scales every torque figure linearly
- The extract flags one small thing itself: `Dowle Rod Small` is set to ABS where the old
  project had it as steel. Worth 2.3 g if it is wrong — negligible, but it is the kind of
  material default that is wrong in a load path somewhere else too.

### ⚠⚠ 180° or 270°? The same pulse range means both

The DS5180SG datasheet: *"Running Degree: 180±3° (PWM 500–2500 µs) **or** 270±3°
(PWM 500–2500 µs)"*. **Both variants take the identical pulse range.** Every angle computed
here assumes 270°, i.e. 7.41 µs/degree. If 180° variants are fitted it is 11.1 µs/degree and
**every derived angle is wrong by 50%**, with nothing in software able to tell.

Settle it on the bench: command a known pulse step, measure the actual sweep with a
protractor. Two minutes, and it invalidates or confirms a lot.

## ⚠ Verify the channel map on every leg before driving it

**Two hip servos were plugged in swapped, and roughly an hour went into commanding the wrong
joint.** The symptom was a "hip lift" that moved the femur left and right, and before that, a
move that produced no visible motion at all — which read as a dead channel.

Nothing in software can catch this. The bus is fine, the registers are correct, the servo
responds. It is only wrong relative to a physical expectation, and **a transposed pair makes a
leg move *wrongly* rather than fail** — far harder to notice than a dead channel, and it
survives every electrical check.

**The guard: after wiring each leg, sweep one channel at a time and name the joint that
moves**, before anything drives a multi-joint pose. Three minutes per leg, and it catches a
transposition while it is still one connector rather than a finished loom. `tools/` has
`hold.py` and `pose.py` for exactly this.

⚠ And **drop every channel limp before unplugging or reseating a servo.** A servo reconnected
to a channel that is still driving snaps to that position the instant it makes contact —
which is precisely when there are hands in the mechanism.

This is the third fault on this bench that presented as a software problem and was not: a bent
ground pin, an unpowered servo rail, and now a transposed pair. **Suspect the physical layer
first.**

## Servo channel map

Fixed convention, the same for all six legs — **leg *N* occupies channels `4N` … `4N+3`**:

| offset | joint | axis | servo |
|---|---|---|---|
| +0 | hip X — coxa yaw | vertical | DS5180SG, 98 kg·cm |
| +1 | hip Y — hip lift | horizontal | DS5180SG, 98 kg·cm |
| +2 | femur joint (`knee` in the CAD extract) | horizontal | DS5180SG, 98 kg·cm — **was 25 kg until leg v2** |
| +3 | tibia joint (`ankle` in the extract) | horizontal | DS3225MG, 25 kg·cm — the binding joint |

24 servos across two PCA9685 boards, so the split falls out of the arithmetic:

| board | address | channels | legs |
|---|---|---|---|
| 0 | 0x40 | 0–15 | legs 0, 1, 2, 3 |
| 1 | 0x41 | 0–7 | legs 4, 5 (8 spare channels) |

Deriving the channel from `(leg, joint)` rather than storing a lookup table is deliberate:
a hand-maintained map is one more thing that can disagree with the loom, and with 24 servos
a single transposed pair is a leg that moves wrongly rather than a leg that fails to move —
much harder to notice.

⚠ **Extended servo leads are a real failure surface.** Each added crimp is a joint that can be
marginal rather than open, which shows up as a brown-out under load rather than as a dead
channel — and a dead channel is far easier to diagnose. The first fault on this bench was a
**bent ground pin**: signal present, power present, no return path, and it looked exactly like
a software bug. Probe each newly-wired channel on its own before it joins a multi-joint move.

## Measured calibration points

The bridge between the model's degrees and the bench's microseconds. Servos confirmed as
**270° variants**, so **7.41 µs/degree** across the 500–2500 µs range.

| leg 1 joint | channel | pulse | pose | measured |
|---|---|---|---|---|
| **coxa yaw** | **0** | **1560 µs** | **femur square to the body** (90° out) | 2026-09-22, speed square |
| coxa yaw | 0 | — | left/right motion | confirmed on hardware |
| **hip lift** | **1** | **higher µs = UP** | femur raises with increasing pulse | confirmed on hardware |

⚠ **This datum belongs to the YAW joint, not the lift** — and that it was ever attributed
to the lift is the story below. "Square to the body" is a yaw concept; a lift axis has no such
position, only up and down. The description did not match the joint, and that was the tell.

⚠ **Datum is "square to the body", not "horizontal to the bench".** They are not the same:
with the leg on a bench the tibia props the rig up, tilting it, so bench-horizontal depends on
how the thing happens to be clamped. Perpendicular-to-body is a property of the robot and is
the one that survives being unclamped.

⚠ **Method note, learned the hard way.** Stepping through a sequence and asking which stop
looked right does not work: the operator is at the bench watching the leg, not reading a
terminal, so the stops are unlabelled and have to be *counted* — which introduced an
immediate 1560-vs-1590 ambiguity over whether "the third move" meant the third position or
the third transition. **Hold ONE pose and ask a yes/no question.** `tools/` has `hold.py`
(one channel) and `pose.py` (several at once) for exactly this.

## Horn indexing is a design parameter (2026-09-21)

**Mount every servo horn at an angle, indexed toward the range the joint actually uses.**

A 270° servo has a fixed travel budget, and where that budget sits relative to the joint is
set once, mechanically, when the horn goes on the spline. Centring the horn on the joint's
neutral spends half the budget on each side — which is only correct if the joint works
symmetrically about neutral. This one does not.

**The robot is either walking or folded. It is never lying flat with a leg fully extended.**
So travel reserved for full extension is travel spent on a pose that never occurs, and it is
exactly the travel needed at the other end to stow a limb.

Verified on the bench: with the horn centred, the ankle joint swung cleanly through its arc
but **could not reach a fold-back** — the servo ran out before the joint did. Re-indexing the
horn a few degrees toward the fold produced **a near-complete fold back, enough for the joint
to serve as a knee**, at the cost of extension range that has no use.

Consequences worth carrying:

- **Per-joint travel is asymmetric about the servo's mechanical centre**, deliberately. A
  spec that assumes `angleMin`/`angleMax` straddle neutral evenly will be wrong for every
  joint on this machine.
- **The horn index is not trim.** Trim corrects a unit's manufacturing zero and is small;
  the index is a design decision worth tens of degrees. Conflating them means a
  recalibration can silently undo a design choice.
- **Decide it before replicating.** This is one mechanical choice repeated 24 times. Getting
  it wrong is not a software fix.
- ⚠ **The fold, neutral and usable-extension pulse values are not yet recorded.** They are
  the first three numbers per joint that would not be estimates. Capture them on the next
  bench session.

## Bench findings — hip v0.1 (2026-09-20)

First leg, first-draft prints, clamped to the bench by a draft hex body. All four servos wired.

**Coxa yaw range is bounded by adjacent-leg clearance, not by the frame.** With stand-in
servos in the neighbouring hip slots, yaw contacted a neighbour well before anything else.
That couples two parameters previously treated as independent: **hip circle radius and usable
yaw range are the same knob.** Legs at 60° on a circle of radius R have a collision angle set
by R and the leg's width at the hip, so widening yaw travel means a bigger circle or a
narrower hip — not a software limit change.

This matters for the stair case specifically. Footholds are 331 mm apart and ±30° of yaw at
R ≈ 120 mm yields only ~120 mm of tangential stride, so a single stride cannot reach the next
tread. The remedies are more yaw or a larger circle, and the bench says those are one remedy.

**Hip lift gave a clean 180°** under full gravity load — the whole leg, both 45 kg servos and
a 350 mm tibia on a long arm — with no rail sag at any checkpoint, on extended servo leads.

**The hip joint then snapped**, at a weak point already identified before the test. That is a
functional test doing its job on a first-draft part. v0.2 reinforces exactly that location.

> ⚠ **Servos have range limits, and the joint's clearance is not the actuator's travel.** A
> 270° servo reaches 270° of whatever the joint allows, so designing 360° of clearance buys
> self-collision freedom, not rotation. Commanding past the servo's travel drives it into its
> own internal gearbox stop.
>
> ⚠ **Stalling is the damage mode, not over-travel.** A servo driven into an immovable object
> draws hard with no motion and heats fast. It is why range finding here uses **expanding
> sweeps** — out, touch, immediately back — so contact lasts one step rather than however long
> a one-way sweep takes to finish. Never dwell against a stop.

### What v0.2 changes, and a consequence worth tracking

In v0.1 the lift servo sat **on top of** the yaw servo, jutting out half its width. v0.2
**stacks** them. Two things follow:

1. **The lateral offset between the yaw and lift axes shrinks or vanishes.** That is
   `coxaLateralOffsetMm` in the spec, carried deliberately as a non-zero-capable parameter
   because printed brackets usually have one and retrofitting the offset-shoulder correction
   later invalidates every pinned number. A stacked hip makes the 0.0 default more likely to
   be literally true — which simplifies the coxa closed form.
2. **A narrower hip buys back yaw range**, via the neighbour-clearance coupling above. So the
   change that fixes the break may also recover some of the stride the stair case is short of.
   Worth measuring once v0.2 is on the bench.

## Three actuation constraints

1. **24 joints, so two PCA9685 boards** at 0x40 and 0x41 (bridge A0 on the second) for 32
   channels. See [pi-bringup.md](pi-bringup.md) for identifying them on the bus.
2. **Run the PWM frame at 200 Hz. Verified on hardware 2026-09-21** — four DS5180-class
   servos driving a real leg, visibly smoother than the same motion at 50 Hz, no brown-out.
   The PCA9685's 12 bits divide the *period*, so 200 Hz (prescale 30, actually 196.89 Hz)
   gives a 1.24 µs tick against 4.88 µs at 50 Hz:

   | frame | tick | angular step | steps/sec |
   |---|---|---|---|
   | 50 Hz | 4.88 µs | **0.61°** | 50 |
   | 200 Hz | 1.24 µs | **0.151°** | 200 |

   ⚠ **Measure angular step, not "ticks per update".** Raising the frame scales the tick
   size *and* the update rate together, so ticks-per-update barely moves and suggests
   nothing improved. What changes is the size of the quantum in degrees, which is what an
   eye can actually see — 4× finer here.

   **Thermal, measured 2026-09-21:** all four servos at **74.2 °F / 23.4 °C — room
   temperature, and identical to each other** after a 20 s coordinated run. That settles
   the frame rate: a servo's internal loop runs 4× as often at 200 Hz, and if that cost
   real current it would show even unloaded.

   ⚠ Scope it honestly — it does **not** establish thermal safety under load. Twenty
   seconds on a free-hanging leg is very little energy, and an IR gun reads the *case*
   while the windings and driver FETs are what fail; case temperature lags internal
   temperature badly. The real thermal case is holding a loaded stance, which this robot
   is designed never to do (see the never-stands-still rule) — so the worst case is
   *walking*, which is duty-cycled by nature.

   **Four identical readings is a free mechanical cross-check** worth repeating routinely.
   A joint that binds, is misaligned, or is partially stalled makes its servo do more work
   than its neighbours, and an IR sweep catches that when the motion still looks fine.

   ⚠ **Quantization shows up where motion is SLOWEST.** A sine is fastest at its zero
   crossing and slowest at its peaks, so a sine-driven joint looks smooth mid-stroke and
   steps visibly at the turnarounds. Real gait is mostly constant-velocity stance, which is
   the regime where this is least visible — so a sine test overstates the problem.
3. **Phase-stagger the channel ON counts.** With every channel starting its pulse on the
   same edge, the pack sees a simultaneous current surge 16× per period. Stagger them
   across the period instead. ⚠ A PCA9685 brown-out reset is **silent** — `MODE1` returns
   to 0x11 with `SLEEP=1`, every output stops, nothing is reported, and the robot simply
   goes limp. Poll `MODE1` and treat `SLEEP=1` as a fault.

> ⚠ **`OE` on the PCA9685 is an active-LOW output *enable*.** `OE` low means outputs are
> driven; `OE` high means they fall back to the `MODE2.OUTNE` state. **The fault action is
> therefore `OE` HIGH.** Getting this backwards arms the outputs in exactly the case the
> fail-safe exists for. Set `OUTNE = 00` so the disabled state drives the outputs low —
> high-impedance lets a long servo lead float and pick up noise a servo may read as a pulse.
>
> And check which way `OE` idles with the microcontroller disconnected. If it idles low,
> a dead controller leaves every servo armed, and the fix is a pull-up so the default is
> disarmed and the controller must actively pull low to arm.
