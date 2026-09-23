# Leg stress test

A repeatable bench test that puts a measured torque through every joint of a leg and
reports where it fails — or proves it does not. Run it on every structural revision
**before** replicating a leg.

Equipment: a digital scale, a tape measure, and the leg clamped to the same rigid surface
the scale sits on. Nothing else.

## The pass criterion: the servo should be the fuse

**A mount must be stronger than the servo that drives it.** If the part gives first you lose a
print and possibly the parts around it; if the servo stalls first you lose nothing — it simply
stops pushing.

| joint | servo | mount must survive |
|---|---|---|
| coxa yaw, hip lift, **femur joint** | DS5180SG | **> 98 kg·cm** (its 7.4 V rating) |
| tibia joint | DS3225MG | **> 25 kg·cm** |

For context, standing asks roughly **57 kg·cm** at the hip lift. So a mount built to survive the
servo carries standing load with a factor of ~1.7 without anyone having to reason about load
share or safety factors. That is the point of the criterion: it replaces a chain of assumptions
with one number that is a property of the part.

> ⚠ **Leg v2 (2026-09-23) moved a mount from the 25 kg row to the 98 kg row.** The femur joint
> is a DS5180SG now, so its mount has to survive **four times** what it did — and the v0.2-era
> print strategy did not reach 98 kg·cm even at the hip. Three of the four mounts on this leg
> are now held to the 98 kg·cm bar.

## One press loads every joint at once

The useful realisation: a single downward force at the toe applies a moment to **all four
joints simultaneously**, each through its own lever — the horizontal distance from that joint's
axis to the contact point. So one press yields four data points, provided the levers are known.

```
tau_joint = F_scale x g x (horizontal distance from that joint's axis to the toe)
```

Changing the pose changes the *ratio* of those four levers, which is what makes a set of poses
worth running rather than one.

⚠ **Pitch-axis levers are X-offsets, not straight-line distances.** The three pitch axes lie
along Z, so for a vertical load only the X component produces torque. Using the straight-line
distance overstates every lever.

## Pose set

Each pose emphasises a different part of the structure. Measure the lever to **every** joint in
each, not just the one being emphasised.

| # | pose | emphasises | why it matters |
|---|---|---|---|
| **A** | femur extended, toe far outboard | **hip lift in bending**, long lever | the standing case, and the one that broke v0.2 |
| **B** | tucked — toe close under the hip | hip mount in **shear and compression** | high force, low moment: a different failure mode entirely |
| **C** | femur folded, femur joint far from the toe | **femur-joint mount and the 330 mm bone** | the longest link carrying the largest bending moment, and a mount newly held to 98 kg·cm |
| **D** | tibia steep, toe just below the tibia joint | **tibia-joint mount**, short lever | the only joint still at 25 kg·cm, and the one the torque budget now binds on |
| **E** | knee-walk — contact at the femur-joint pad | **femur in compression**, hip near zero torque | the intended high-load mode; should be the *easiest* pose |
| **F** | leg near-vertical, toe under the hip | links in **pure compression** | tests the bones rather than the joints |
| **G** | lateral — toe pushed sideways | **coxa yaw**, which carries all propulsion and side load | nothing has ever tested this axis |

Pose G needs the scale on its side or a block-and-scale sandwich. It is the one axis with zero
gravity load and therefore zero coverage from every other test — and it takes the entire
horizontal reaction during a step.

## Procedure

1. **Zero the scale empty**, then place it. Zeroing under load hides the limb's own weight,
   which is part of the force that produces torque.
2. Pose the leg with every joint **energised** — a limp joint collapses instead of loading.
3. **Tape the lever** to each joint axis. If the toe happens to sit level with a joint's axis,
   the straight-line distance *is* that joint's lever; otherwise subtract the height difference.
4. Step the driving joint toward the scale in **~10 µs increments (1.4°)**, holding ~7 s, and
   record the reading at each step.
5. Stop at a plateau, at the target torque, or at failure.

**Proof-load (non-destructive), for legs you intend to keep:** load to the pass criterion, hold
30 s, release, and check for permanent set — a joint that no longer returns to its pose, a
whitened crease, or a changed resting angle. **Destructive, for one sacrificial print per
revision:** keep going to failure and photograph the fracture.

## ⚠ The failure signature — it does not look like failure

When v0.2's hip-lift mount failed, the numbers looked like a weak servo:

- **Force plateaued** at ~0.3 kg and then drifted *down*
- **~300 mm of commanded rotation went missing** — the servo turned, the toe did not move
- **Servos stayed at room temperature**

All three together mean the structure is yielding, **not** that the actuator is at its limit.
A stalling servo pushes hard and gets hot. A failing mount absorbs the travel and stays cool,
because the load never reaches the servo at all. Plateau + missing travel + cool servo = look
at the structure.

## Baseline: hip v0.2, 2026-09-22

Pose A. Toe initially level with the hip-lift gear at **420 mm**, so that was the lever directly.

| | |
|---|---|
| failure | hip-lift servo mount, snapped in three places |
| force at failure | ~0.30 kg at the toe |
| **hip torque at failure** | **~5–11 kg·cm** (lever 156–381 mm over the loading range) |
| vs standing requirement | ~1/5 |
| vs servo rating | ~1/10 |

**Cause was print strategy, not material or geometry.** The mount had its side panels hollowed
to save filament, leaving a cage of 10 × 5 mm supports over an X of sparse infill. It failed at
the supports. So this figure is a floor for *that print*, not a property of the design — and
PETG-CF or GF plus solid material through the load path should move it by a large multiple.

**Target for v0.3: > 98 kg·cm at the hip lift**, i.e. roughly ten times the v0.2 result, at
which point the servo becomes the fuse.
