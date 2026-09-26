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

### Knee contact is *the* load-bearing mode, and it is feasible by construction

The pad sits at the **tibia joint** — the bend at the bottom of the femur, which the CAD
extract confusingly calls `ankle`. Fold the tibia back and that joint becomes the lowest point.

Three things then fall out, all of them good:

1. **The tibia joint's lever goes to ~0**, because the contact is on its own axis. Its torque
   is the pad's offset from the axis, not a link length. The 25 kg servo is out of the load
   path entirely.
2. **Only two links stand between the hip and the ground** (112 + 330 mm), so the hip lever
   cannot exceed 442 mm geometrically and is capped at 162 mm by the budget — the same cap as
   plantigrade. **No stance width is lost.**
3. **The femur joint's lever shrinks as the pose gets taller**, because a taller pose means a
   more vertical femur:

| body height at 160 mm offset | hip lever | femur lever | tripod tipping |
|---|---|---|---|
| 200 mm | 160 mm (99% of cap) | 124 mm (76%) | **30.2°** |
| 250 mm | 160 mm (99%) | 74 mm (46%) | 25.0° |
| 300 mm | 160 mm (99%) | 51 mm (32%) | 21.2° |
| 400 mm | 160 mm (99%) | 82 mm (51%) | 16.2° |

The hip is the joint that works in this mode and the femur joint has margin everywhere. With
the unloaded tibiae splayed outward as outriggers the tipping angle goes far past any of these,
since an outrigger costs almost nothing — 39.5 g of tibia at a 0.53 kg contact rating.

> ⚠⚠ **THE KNEE PAD MUST TRANSFER LOAD INTO THE FEMUR, NOT INTO THE TIBIA SERVO'S MOUNT.**
> The pad location and the DS3225MG's bracket are the same place on the part. Bolt the pad to
> the bracket and the entire weight of the robot goes through the one printed mount holding the
> smallest servo — the exact part class, and very nearly the exact part, that failed at
> ~5–11 kg·cm on v0.2. The pad is a structural member of the femur that happens to sit beside a
> servo, and it should be printed and loaded as one.

> The tibia is also **free while standing**. Six folded-back tibiae with toes rated for half a
> kilo are six light manipulators available without leaving the load-bearing pose — which is the
> same observation that makes the stored-solar-panel stretch goal unexotic.

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

### The tibia does not carry the robot — the knee pad does

**Design intent (Ben, 2026-09-23): the tibia and toe are for reach and posturing. Anything that
would stress them — supporting full weight — is done by flipping the tibia back and standing on
a pad at the knee.** That single sentence resolves what otherwise reads as an under-sized servo,
and it is why the numbers below are a *specification* rather than a problem.

Putting an 80 kg servo on the femur joint handed the nominal torque constraint to the one joint
still on a DS3225MG, which drives the longest link that reaches the ground, 234.81 mm of it.

In-budget levers, DS5180SG derated to **90.6 kg·cm at 6.6 V**, DS3225MG at **25 kg·cm**:

| M | share | SF | coxa / hip / femur | **tibia** |
|---|---|---|---|---|
| 5.3 kg | ideal ⅓ | 2.0 | 256 mm | **71 mm** |
| 5.6 kg | ideal ⅓ | 2.0 | 243 mm | **67 mm** |
| 5.6 kg | worst 0.5 | 1.5 | 216 mm | **60 mm** |
| 5.6 kg | worst 0.5 | 2.0 | 162 mm | **45 mm** |
| 5.9 kg | worst 0.5 | 2.0 | 154 mm | **42 mm** |

Read as a weight-bearing joint that would say the tibia has to stay within ~11° of vertical.
Read correctly — as a **light contact** — it says how much the toe may take:

| tibia attitude | lever | max toe force | with SF 2.0 |
|---|---|---|---|
| horizontal (flat-foot) | 234.8 mm | 10.4 N | **0.53 kg** |
| 60° from vertical | 117.4 mm | 20.9 N | **1.06 kg** |
| 80° from vertical | 40.8 mm | 60.1 N | **3.06 kg** |

**The toe is a finger, not a foot.** Half a kilo at full extension, three kilos when nearly
under its own joint. That is ample for what it is for: probing a surface before committing to
it, feeling for ground during swing, bracing against a riser, splaying as an outrigger,
placing an object. It is not ample for a sixth of the robot, and it is not meant to be.

**Keep the DS3225MG.** Its real job is swinging an unloaded tibia: the whole tibia assembly is
**39.5 g** with its mass centre ~117 mm out, which is **0.46 kg·cm — 1.8% of a 25 kg servo**.
Fitting one of the spare 80s there would add 105 g per leg, **630 g on the robot**, to a joint
running at under two percent of the part already in it. Mass is the most leveraged number in
the whole torque budget; this is the wrong place to spend it.

### Two envelopes, one width

All three 80 kg joints cap at the **same 162 mm** of horizontal offset, because each one's
lever is the horizontal distance from its own axis to the contact and the hip's is always the
largest. **That cap is the same in both contact modes**, so knee-stand gives up no stance width
at all — 162 mm of offset plus a 72.7 mm hip circle is a foot radius of 235 mm, span ≈ 469 mm.

What the modes differ in is **height**:

| | body height range | what it is for |
|---|---|---|
| knee-stand (pad at the tibia joint) | **146 – 414 mm** | carrying the robot |
| plantigrade (toe down) | up to **645 mm** | reaching, posturing, probing |

So the leg has a weight-bearing envelope and a taller reach-only envelope stacked on top of it,
which is exactly the split Ben designed for. Reach is what versatility is made of; the load
path is separate and shorter.

Stability is one-sided in both — spend nothing on height you do not need:

| body / COM height | tripod tipping | 5 legs down |
|---|---|---|
| 200 mm | **30.7°** | 43.9° |
| 250 mm | 25.4° | 37.6° |
| 300 mm | 21.6° | 32.6° |
| 400 mm | 16.5° | 25.7° |

Standing low is worth more than a stronger servo. And the knee-stand envelope tops out at
414 mm anyway, so the load mode is *naturally* in the stable part of the range.

### ⚠ Hip circle radius is free stability — set it before the six-leg assembly exists

Foot radius is `hip_circle_radius + stance_offset`. The torque budget constrains only the
second term; the first is carried by the coxa yaw axis, which is **vertical and therefore
carries no gravity torque at all**. So body radius buys stance width one-for-one at zero cost
to any servo:

| hip circle | foot radius | span | tripod tipping @ 250 mm |
|---|---|---|---|
| 72.7 mm (stale v1 value) | 233 mm | 465 mm | 25.0° |
| 100 mm | 260 mm | 520 mm | 27.5° |
| **120 mm** | **280 mm** | **560 mm** | **29.2°** |
| 150 mm | 310 mm | 620 mm | 31.8° |

At a 120 mm hip circle the low knee-stand pose (200 mm body height) reaches **35.0°** of tipping
angle — past the 32.5° a continuous stair ramp would demand, which the old geometry missed by a
factor of two. It costs body plate, some wiring run and a little body mass. **There is no
six-leg assembly yet, so this is the cheapest it will ever be to change.**

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

Footholds on a staircase are 331 mm apart and ±30° of yaw at R ≈ 120 mm yields only ~120 mm of
tangential stride, so **one stride cannot reach the next tread.** That is not the failure it
first looks like — the goal is that the robot *works out how to get up*, not that it takes
stairs in stride, so multiple steps per tread with a body shift between them is the expected
answer rather than a workaround. It does say the hip circle wants to be bigger, and that is now
the third independent argument for the same knob: yaw clearance, stance width, and free
stability all improve with it, and none of them costs a servo anything.

**Hip lift gave a clean 180°** under full gravity load — the whole **leg v1**, both 25 kg
servos and a 353 mm femur on a long arm — with no rail sag at any checkpoint, on extended
servo leads. (Leg v2 is heavier and shorter; the number does not carry over untested.)

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

## Bench findings — leg v2 shakedown (2026-09-23)

A short unplanned load test. The leg was hand-posed on the bench, hip clamped, toe on the
scale; three channels were energised at 1500 µs to establish a reference pose. **1500 µs is
mid-*pulse*, not mid-*pose***, so the joints snapped from where they had been set to wherever
1500 put them, which drove the leg down onto the scale and then, once de-energised, let it
collapse flat with the tibia servo resting on the scale centre.

**Nothing broke.** That is the first structural result for the v2 prints, and it is a much
harder hit than the controlled v0.2 run that snapped a hip mount at ~0.30 kg. Ben: *"everything
held up and it was a good test — we want to push it harder each run. If six legs did that with
a free body it would have stood up."* Which is the right read: an unloaded bench leg driving
itself into a fixed surface is close to the load a leg sees when the body is the thing that
moves instead.

Three things worth keeping:

1. **⚠⚠ A de-energised leg does NOT hold its pose.** The assumption that big metal-gear servos
   would hold position by gear friction with power off is **wrong on this hardware** — the leg
   went flat. This is the first hardware evidence bearing on `PARK.stableWithTorqueOff`, which
   is the central invariant of the machine: park is the rest state, the charge state and the
   failure state at once, and on a light pack it is also the docking state. **Park must be
   mechanically stable — resting on structure, skids or a detent — and can borrow nothing from
   the servos.** Design the chassis underside accordingly; do not plan to "leave it where it
   stops".
2. **It collapsed onto the tibia servo, which is exactly where the knee pad goes.** The pad
   location and the DS3225MG's bracket are the same place on the part, and tonight the leg's
   weight went through that bracket. At 730 g it is fine; at a sixth of 5.6 kg it is the load
   path the pad exists to avoid. See the knee-contact section: the pad is a structural member
   of the femur that happens to sit beside a servo.
3. **Hip-lift direction is probably inverted on leg v2.** Ben's read of the motion: *"it looked
   like the leg was floundering on the ground and thought you meant to be lifting instead of
   going down."* Leg v1's datum was *higher µs = UP*. Treat v2 as **unknown until re-measured**;
   the direction probe run that evening was inconclusive because the pose it started from was
   itself a guess.

> **⚠ Procedure, and this is the lesson that cost the evening: never make the first energise of
> an unknown pose while the toe is loaded.** A PWM servo has no feedback and no soft start, so
> the first pulse is a snap to the commanded position from wherever the joint happens to be.
> Unload the contact first — slide the scale out, let the leg hang — energise one channel at a
> time, converge on a pose the operator confirms by eye, *then* put the load back. The bench
> rule was already written down ("hand-set joints near mid-travel before energising") and was
> not followed.

**Still owed: leg v2's home pose in microseconds.** The new Fusion assembly has no user
parameters, so nothing in the model defines joint zero or direction against pulse width. Find
it the way the calibration section prescribes — hold ONE value, ask higher or lower, converge —
with the leg unloaded, and record it in the calibration table.

## ⚠⚠ Bench findings — first powered runs (2026-09-25/26)

The session that turned the camera into an instrument. Four joints, one of which died, and
one finding that invalidates a tier of the safety architecture.

### ⚠⚠ THE SERVOS HOLD POSITION AFTER THE SIGNAL STOPS — "no pulse" is NOT "no torque"

The single most consequential result on this page. Measured on the bench supply with every
channel read back as `FULL_OFF`, i.e. the PCA9685 generating no pulses at all:

| state | rail current |
|---|---|
| cold, nothing ever commanded | **0.023 A** |
| after ch0/ch1/ch2 held once, then released | **0.239 A** |

216 mA across three servos that are being sent nothing — ~72 mA each, an order of magnitude
above any plausible quiescent draw. Confirmed by hand, which is the better instrument here:
**Ben cannot force those joints at all while the rail is live**, signal or no signal.

⚠ **Unpowered is NOT limp either, and this corrects the 2026-09-23 note.** With the rail off
the gearbox friction holds the leg's pose — Ben can move it by hand, hearing the gears turn,
but it does not collapse. So there are *three* states, not two:

| state | behaviour |
|---|---|
| rail off | friction holds the pose; back-drivable by hand with effort |
| rail on, no signal | actively holding; cannot be forced |
| rail on, commanded | actively holding; cannot be forced |

**The friction hold is load-dependent and must never be relied on.** On 2026-09-23 the same
leg went flat when de-energised — with the tibia *extended*, roughly double the moment about
the hip. Folded back it holds; extended it did not. Friction that holds 730 g of leg says
nothing about 5.6 kg of robot, so `PARK.stableWithTorqueOff` still has to be satisfied by
structure.

**The plan's reflex tier assumes the opposite.** `OE` high, the 0x70 all-call stop and the
`MODE2.OUTNE` choice all stop *pulses*, and the design treats that as removing torque. On
this hardware it does not. Consequences that must flow into the design:

- **`emergencyTorqueOff()` needs a POWER path, not a signal path** — a relay or high-side
  MOSFET on the servo rail, driven by the same reflex tier that drives OE. OE alone leaves
  24 servos energised and holding.
- **`requestTorqueOff()` / `bellyDrop()` cannot end by simply ceasing to command.** The
  robot will stand there holding, drawing current, until V+ goes away.
- It cuts the other way too, and in our favour: **a signal loss does not drop the robot.**
  A Pi crash mid-stance leaves it standing rather than collapsing, which is the failure mode
  you would have designed for if you could.
- The OE-idle-direction question (pull-up vs pull-down, still unmeasured) matters *less*
  than assumed for disarming, and not at all for making the machine go limp.

### ⚠ The MODE1 brown-out detector is blind to the servo rail

`MODE1.SLEEP` is the documented silent-brown-out canary and it works — for the *logic*
supply, which on this breakout comes from the Pi's 3.3 V. **V+ feeds only the servos.** MODE1
will read a cheerful `0x21` straight through a total collapse of the servo rail. Nothing in
the I²C path can see that rail; it needs its own sensor (the ADS1115 already suggested, or
the bench supply's own display while we are on a bench).

### Two bugs in our own tooling, both of which faked a hardware fault

1. **`set_counts()` masked the OFF high byte with `0x0F`.** `FULL_OFF` is **bit 4 (0x10)**,
   so the mask deleted exactly the bit that stops the output and `channel_off()` was a
   silent no-op all session. Mask is `0x1F`. Found because the bench supply read a
   **cumulative** current across sequential single-channel holds — each servo stayed live
   after "its" command ended.
2. **`Leg.off()` called `all_off()` FIRST and then sixteen per-channel writes**, which
   overwrote the stop it had just issued. It undid itself. `all_off()` goes last.

Both meant the dwell-bounded safety property the calibration tool advertises was not real
until it was fixed. **A safety guarantee that has never been tested is a comment.**

### The camera is a real instrument now — and here is what it cannot do

A BRIO on kraken, aimed square at the leg, closed the loop: joint motion is read off frames
rather than narrated by the operator. What it bought, immediately: a sweep that produced
**no movement at all** was detected as such, and "nothing happened" is precisely the result
that is invisible when a human is watching, because it looks identical to "I wasn't looking
at the right moment."

- **Frame differencing beats eyeballing.** PSNR against a reference frame: ~20 dB when the
  tibia visibly swung, 33–36 dB when it did not move at all. A number, not an impression.
- **Colour mask on SATURATION, not hue.** Under incandescent shop light the brick, bench and
  skin all read orange. The filament measured **S=176** against 53 (brick) / 67 (bench) /
  104 (dark ceiling), and R/G of 3.1 against 1.1–1.3. Saturation separates; hue does not.
- **A speed square in frame is the obliquity check, and it is worth more than scale.** Its
  90° corner photographed as **90.37°**, so the camera was square to the leg's swing plane
  to within half a degree and every measured angle is real. Without it an oblique view
  under-reads every angle and would report a 270° servo as a 180° one with total confidence.
  Scale comes free from the known link lengths; the right angle is the thing you cannot get
  any other way.
- ⚠ **A single still frame CANNOT see a motion transient.** An unloaded servo's holding
  current is near zero and its motion surge lasts ~0.2 s, so a frame taken a second into a
  7 s hold reads idle. This produced a confident, wrong conclusion that a working channel
  was dead. Capture **video** and read the peak, or test under load.
- ⚠ **The bench supply's display averages and updates a few times a second.** Its readings
  are valid for steady-state holds and meaningless for transients.
- ⚠ **A BRIO exposes four `/dev/video*` nodes and only the first is the camera** (the third
  is a 340×340 greyscale IR sensor), and **the first frame is underexposed** — 67 KB versus
  368 KB for the same scene twelve frames later. Both failures return a plausible image.
  `tools/bench-cam.sh` refuses any node that does not advertise mjpeg and discards warmup
  frames.

> ⚠ **A fit statistic cannot tell you it measured the wrong thing.** A least-squares circle
> through five tracked "toe" positions returned a 0.24 px residual — a beautiful number —
> and every one of those points was the bottom edge of the crop box, not the toe. Five
> nearly-collinear points fit almost any large circle. Same family as the photo-path check
> that reported "212/212 covered" by verifying each chunk against itself.

### Calibration does not survive reassembly

A horn re-index or a swapped connector silently invalidates the whole µs↔angle map. The
"rising µs retracts the tibia" result was measured, then the leg was rebuilt, and it could
no longer be assumed. **Any mechanical change to a joint means re-running its sweep before
anything is commanded against a load.**

### Structural results — the plumb strut is not what will break

| test | result |
|---|---|
| press down on the plumb femur, contact centred | **10 kg**, no fold |
| pull down on the hip arm until it felt close to failure | **5 kg** |

For scale: the machine is 5.3–6.0 kg, a tripod puts ~2 kg on a leg, and the v0.2 hip mount
snapped at **0.30 kg**. That is 5× the walking load on one leg and ~16× the old failure
point in the opposite direction.

### Geometry confirmed by tape, and the CAD was right

| | value |
|---|---|
| hip-lift shaft → femur shaft | **110 mm** (CAD says 111.95 — vindicated) |
| femur shaft → ground contact | **340 mm** |
| fully outstretched, hip-lift shaft → contact | **450 mm** |

An earlier "19 cm lifting arm" was the arm *including the servo body*, not shaft-to-shaft.
Nothing downstream of `leg-geometry.json` needed revisiting.

**The plumb stance is entirely inside budget at every height.** Hip lever is `L2·cos(arm
angle)`, so it peaks at the arm's own length:

| lift arm | hip lever | body height | load at SF 2.0 | at stall |
|---|---|---|---|---|
| straight up | 0 mm | 230 mm | — | — |
| 60° up | 55 mm | 245 mm | 8.2 kg | 16.5 kg |
| horizontal | **110 mm** | 340 mm | 4.1 kg | 8.2 kg |
| 60° down | 55 mm | 435 mm | 8.2 kg | 16.5 kg |
| straight down | 0 mm | 450 mm | — | — |

Worst-case hip lever anywhere in that envelope is **110 mm against a 162 mm cap**, and the
220 mm of vertical travel clears a 178 mm stair riser *in the load-bearing stance*. **Do not
lengthen the lift arm**: 190 mm would buy 380 mm of travel at a 190 mm worst-case lever,
over the cap, exactly at the mid-height where the robot wants to stand. Shortening is the
direction that risks the plumb femur fouling the hip.

The **triangle pose** (femur lever 335 mm measured) is a reach-and-place pose, not a
weight-bearing one: 1.35 kg at SF 2.0 at the femur joint, and the hip is worse. Do not load
it.

### Bench power

**7.4 V rail, 5.0 A limit.** 7.4 V is the 2S pack's nominal, so bench numbers transfer to
the machine at mid-charge; derate ~7% toward the 6.6 V cutoff (90.6 vs 97.4 kg·cm for the
DS5180SG). Set the current limit by shorting the leads, setting CC, then removing the short.

- ⚠ **A too-low current limit is a confusing fault, not a safe one.** With the supply in CC
  the rail simply cannot deliver, every reading pins at the limit, and because MODE1 is
  blind to V+ nothing reports it. Watch the CV/CC indicator.
- ⚠ **The LM2596 buck is a ~3 A part** and a DS3225MG stalls at ~2.5–3 A, so it is marginal
  exactly when the tibia works hard — and a buck folding back under a surge is
  indistinguishable from a dead servo. It also costs ~30 mA of quiescent draw (idle went
  0.023 → 0.053 A when fitted). Its real job is 0.46 kg·cm, so it will never legitimately
  need that current; treat "tibia went dead" as "check the buck first", or fit a 5 A module.

### The tibia fault was the buck and the splices — the servo was fine all along

Symptom: zero current and no motion on ch3, through a new PCA9685, bonded grounds, a
verified 6.0 V buck output and a correct pulse confirmed by register read-back. The
diagnosis cost an evening and produced two wrong conclusions before the right one.

- A **brand new 25 kg servo plugged straight into ch3 worked on the first try**, so the
  channel was never at fault — and the original servo, once the wiring was cleaned up,
  **works too**. Nothing was broken. It was the buck and the splices between them.
- ⚠ **"Draws no current" did not mean "not driven".** An unloaded servo's holding current is
  ~0 and its motion surge is ~0.2 s, so the bench supply — which averages and updates a few
  times a second — showed idle for a channel that was working perfectly. The operator's eyes
  caught in one second what the instrument could not see at all.
- ⚠ A supply whose **current limit is set too low** produces the same symptom by a different
  mechanism: every reading pins at the limit and nothing reports it. Both were live
  hypotheses at once, and both were wrong.

The lesson that generalises: **a measurement that reads "nothing" is the weakest possible
evidence**, because every failure of the instrument also reads "nothing". Confirm a negative
against a second, independent channel before acting on it.

⚠ **The tibia attachment has now failed three times in one day** — it collapsed under the
leg's weight, it was re-indexed on the horn, then it came off the horn during an unloaded
sweep. It is the weakest interface on the leg. That is tolerable only because the design
deliberately keeps the tibia out of the load path; it is *not* tolerable for the
tibia-as-finger work, which loads that exact joint.

### ⚠⚠ THE HIP ARM FRACTURED during an uncalibrated travel sweep (2026-09-26)

Driving ch1 (hip lift) from 1600 to 1700 us broke the printed lift arm. The leg was airborne,
so self-weight was only ~10 kg·cm — nowhere near a failure load. The servo drove the arm into
its own mechanical limit and kept pushing: **up to 98 kg·cm into a printed part, held for the
full 7 s dwell.**

**The procedural failure is precise and it is worth more than the part.** The bench supply's
current is the one signal that separates *moving* from *pushing against a stop*. It had been
in frame all evening and was the instrument that caught two earlier faults — and then the
angle measurements moved to the profile camera, which cannot see the supply, and the meter
silently dropped out of the loop. So a travel sweep ran on a joint whose travel limits were
unknown (the entire purpose of the sweep) with **no abort criterion**. The same hazard had
already been written down one joint earlier — *"the tibia is folded against the femur, so the
femur is a hard stop for that joint"* — and was not carried across to the hip.

**Rules this buys:**
- **A travel sweep must monitor current, not just position.** Rising current with no angle
  change is a stall; abort on it. One camera cannot watch both the leg and the meter, so
  either both cameras fire per step or the rail gets its own sensor (the ADS1115 already on
  the list — see the MODE1 blind-spot note above).
- **Shorten the dwell when probing UNKNOWN travel.** 7 s is right for settling a pose that is
  known to be reachable; it is far too long for a value that might be past a stop. ~1 s is
  enough to photograph.
- **Approach an unknown limit in small steps from a known-good value**, not in 100 us jumps.
- ⚠ **Swapping instruments is a change to the safety system.** Nothing announced that the
  stall detector had been removed, because it was never a component — it was a habit.

**Tooling from this session:** `tools/leg-calibrate.py` (dwell-bounded probe/sweep/hold),
`tools/exercise-channel.py` (hands-free wiggle for wiring troubleshooting, with the three
measurements to take while it runs), `tools/bench-cam.sh` (BRIO capture that refuses the IR
node and discards warmup frames).

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
