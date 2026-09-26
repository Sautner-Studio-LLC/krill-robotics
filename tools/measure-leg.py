#!/usr/bin/env python3
"""Measure leg-v2 joint angles from a photograph, using the coloured tape fiducials.

Every printed part is the same orange and the bench stand is orange clamps, so shape alone
cannot tell the links apart. Coloured tape does:

    green  -> tibia      blue -> femur      magenta -> hip lift arm

⚠ USE THE PROFILE CAMERA. The three pitch joints (hip lift, femur, tibia) all swing in one
  vertical plane. A camera looking ALONG that plane foreshortens every angle in it, and the
  links only look separate because they are laterally offset -- so it produces plausible,
  wrong numbers. On this bench that is camera 1 (/dev/video0). Camera 2 (/dev/video4) faces
  the leg head-on: right for coxa yaw, useless for pitch.

⚠ Markers are found ONLY inside a dilated region around the orange links, or the brick, the
  bench and the background furniture all match.

⚠ A dark marker fails regardless of colour. Measured 2026-09-26: with the ring light raking
  from one side, the lit bar clipped at V=253 while the shadowed bar sat at V=127 and its
  tape crushed to an indistinguishable mud. Light both sides flat, target V~190-210, nothing
  at 255. This tool reports exposure so that is checkable rather than a matter of opinion.

Angles are reported against image vertical, corrected by --roll. Calibrate roll once from a
pose whose geometry is known -- e.g. the toe stand, where the tibia is perpendicular to the
benchtop -- with --calibrate.
"""
import argparse, json, os, sys
import numpy as np
from PIL import Image, ImageFilter

MARKS = {
    "tibia":  ("green",   lambda R,G,B,S,V: (G > R*1.05) & (G > B*1.25) & (V > 80)),
    "femur":  ("blue",    lambda R,G,B,S,V: (B > R*1.10) & (B > G*1.02) & (V > 60)),
    "hiparm": ("magenta", lambda R,G,B,S,V: (R > G*1.50) & (B > G*1.25) & (V > 70)),
}


def load(path, roi):
    im = Image.open(path).convert("RGB")
    if roi:
        im = im.crop(roi)
    a = np.asarray(im).astype(np.int16)
    hsv = np.asarray(im.convert("HSV")).astype(np.int16)
    return im, a[:, :, 0], a[:, :, 1], a[:, :, 2], hsv[:, :, 1], hsv[:, :, 2]


def leg_region(R, G, B, S, V, grow):
    orange = (S > 110) & (R > G*1.6) & (R > B*1.6) & (V > 90)
    grown = Image.fromarray((orange*255).astype(np.uint8)).filter(ImageFilter.MaxFilter(grow))
    return orange, np.asarray(grown) > 0


def blobs(mask, cell=14, min_px=120):
    """Coarse-grid connected components. A tape RING around a cylinder projects as a band
    ACROSS the link, so a single blob's own long axis is PERPENDICULAR to the link and must
    never be used as the link direction. The line through several blobs' centroids is what
    gives the direction, which is why the markers are rings strung along each link."""
    from collections import deque
    h, w = mask.shape
    gh, gw = h // cell, w // cell
    g = mask[:gh*cell, :gw*cell].reshape(gh, cell, gw, cell).sum((1, 3))
    occ = g >= cell*cell*0.22
    seen = np.zeros_like(occ)
    out = []
    for i in range(gh):
        for j in range(gw):
            if not occ[i, j] or seen[i, j]:
                continue
            q = deque([(i, j)]); seen[i, j] = True; cells = []
            while q:
                y, x = q.popleft(); cells.append((y, x))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny, nx = y+dy, x+dx
                        if 0 <= ny < gh and 0 <= nx < gw and occ[ny, nx] and not seen[ny, nx]:
                            seen[ny, nx] = True; q.append((ny, nx))
            sub = np.zeros_like(mask)
            for (y, x) in cells:
                sub[y*cell:(y+1)*cell, x*cell:(x+1)*cell] = mask[y*cell:(y+1)*cell, x*cell:(x+1)*cell]
            n = int(sub.sum())
            if n >= min_px:
                ys, xs = np.nonzero(sub)
                out.append((n, float(xs.mean()), float(ys.mean())))
    return sorted(out, reverse=True)


def axis(mask, tol=35.0):
    """Direction of the link carrying this marker, by RANSAC over blob centroids.

    Outliers are real and common -- a scrap of the same tape on the bench, a coloured tool,
    a reflection. Fitting all marker pixels at once lets one distant blob swing the axis by
    tens of degrees, which is exactly how a vertical tibia first measured as -76.9 deg."""
    bl = blobs(mask)
    if len(bl) < 2:
        return None, int(sum(b[0] for b in bl)), len(bl), 0
    P = np.array([[x, y] for _, x, y in bl])
    W = np.array([n for n, _, _ in bl], float)
    best = None
    for i in range(len(P)):
        for j in range(i+1, len(P)):
            d = P[j] - P[i]
            L = np.hypot(*d)
            if L < 25:
                continue
            nrm = np.array([-d[1], d[0]]) / L
            dist = np.abs((P - P[i]) @ nrm)
            inl = dist < tol
            score = W[inl].sum()
            if best is None or score > best[0]:
                best = (score, inl)
    if best is None:
        return None, int(W.sum()), len(bl), 0
    inl = best[1]
    if inl.sum() < 2:
        return None, int(W.sum()), len(bl), int(inl.sum())
    # weighted total-least-squares through the inlier centroids
    Q = P[inl]; w = W[inl]
    mu = (Q * w[:, None]).sum(0) / w.sum()
    D = (Q - mu) * np.sqrt(w)[:, None]
    _, _, Vt = np.linalg.svd(D, full_matrices=False)
    dx, dy = Vt[0]
    ang = np.degrees(np.arctan2(dx, dy))
    if ang > 90:  ang -= 180
    if ang < -90: ang += 180
    return float(ang), int(w.sum()), len(bl), int(inl.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("--roi", help="x0,y0,x1,y1 to crop before measuring")
    ap.add_argument("--grow", type=int, default=41, help="dilation of the orange leg region")
    ap.add_argument("--roll", type=float, default=0.0, help="camera roll correction, degrees")
    ap.add_argument("--calibrate", choices=list(MARKS),
                    help="treat this link as truly vertical and print the roll it implies")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()

    roi = tuple(int(v) for v in a.roi.split(",")) if a.roi else None
    im, R, G, B, S, V = load(a.image, roi)
    orange, leg = leg_region(R, G, B, S, V, a.grow)

    clipped = int((V[orange] >= 254).sum())
    expo = {"orange_mean_V": round(float(V[orange].mean()), 1), "clipped_px": clipped}

    out = {"exposure": expo, "links": {}}
    for name, (colour, test) in MARKS.items():
        m = test(R, G, B, S, V) & leg
        ang, n, nblob, ninl = axis(m)
        out["links"][name] = {
            "colour": colour, "px": int(n), "blobs": nblob, "inliers": ninl,
            "angle_from_vertical": None if ang is None else round(ang - a.roll, 2),
        }

    if a.calibrate:
        ang = out["links"][a.calibrate]["angle_from_vertical"]
        if ang is None:
            sys.exit(f"cannot calibrate on '{a.calibrate}': marker not found")
        print(f"roll implied by a vertical {a.calibrate}: {ang + a.roll:+.2f} deg"
              f"   (pass --roll {ang + a.roll:.2f})")
        return

    if a.json:
        print(json.dumps(out, indent=1)); return

    print(f"exposure: orange mean V={expo['orange_mean_V']}  clipped={clipped} px"
          f"   {'OK' if clipped < 2000 and 120 < expo['orange_mean_V'] < 235 else '<-- fix the light'}")
    for name, d in out["links"].items():
        if d["angle_from_vertical"] is None:
            print(f"  {name:7s} {d['colour']:8s}  NOT FOUND ({d['px']} px)")
        else:
            warn = "" if d["inliers"] >= 2 else "   <-- too few markers, angle unreliable"
            print(f"  {name:7s} {d['colour']:8s}  {d['angle_from_vertical']:+7.2f} deg from vertical"
                  f"   ({d['px']:5d} px, {d['inliers']}/{d['blobs']} blobs on the line){warn}")


if __name__ == "__main__":
    main()
