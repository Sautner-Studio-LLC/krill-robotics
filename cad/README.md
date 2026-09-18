# CAD

Parts are modelled **by hand in Fusion 360**. There is no generated geometry in this
project and no scripted model pipeline — if you find one, it is stale.

```
cad/src/      *.f3d    editable Fusion source
cad/export/   *.step   the portable deliverable — opens without a Fusion licence
              *.stl    print-ready meshes
cad/vendor/   gitignored — third-party reference bodies, see "Credits" below
cad/wip/      gitignored — in-flight exports
```

**STEP is the format that matters here.** `.f3d` needs a Fusion licence, so a repo that
published only `.f3d` would give you nothing you could open. STEP is neutral, opens in
FreeCAD, Onshape, SolidWorks and Fusion alike, and is plain ASCII so it diffs in git.

## Printed parts

PLA for geometry iteration; PA-CF or PLA-CF for a final set — the win there is creep
resistance under continuous load and heat, not stiffness. Heat-set inserts throughout, no
self-tappers into plastic.

⚠ **Printed parts are not solid.** At ~4 perimeters and ~30 % gyroid, mass is well under
`volume × material density`. Anything computing mass from these solids must apply a
measured infill factor, calibrated by weighing a real print.

## Credits — reference models used, *not* redistributed

Servo solids and similar are used as **visual aids and fit references** while modelling.
They are other people's work under their own licences, so they are **not included in this
repository** and `cad/vendor/` is gitignored. Download them yourself if you want the
assembly to look complete:

- DS-Servo DS5160 SSG — [GrabCAD](https://grabcad.com/library/ds-servo-ds-5160-ssg-1)
- DS5180-class servo — [Thingiverse thing:5678163](https://www.thingiverse.com/thing:5678163)

Thanks to their authors. Everything in `cad/src` and `cad/export` is original work under
this repository's MIT licence; nothing here relicenses anyone else's model.

## Published parts

Printable parts are also published standalone, so you can print them without cloning
anything:

<!-- Add GrabCAD / Printables / Thingiverse links here as parts are released. -->
_None published yet — the first leg is still being iterated._
