"""Rod-free hinge variants for Original/coin.STEP (Kia Sorento coin + mechanical key tray).

The original hinge needs two Ø2.41 mm metal rods pushed through the tray knuckles
into the lid barrel. This script fills those bores and builds three rod-free joints:

  keyed           lid pins with two flats drop into slots in the tray knuckles, only
                  when the lid is held KEY_ANGLE open; a narrow neck makes it click
  snap            round lid pins snap into C-shaped sockets through a narrow throat
  print_in_place  cone pins in cone sockets, printed already assembled, lid open PIP_POSE

Run with the PowerShell `python` (needs cadquery, trimesh, matplotlib; do not use -I):
    python make_rodless_hinge.py                 # all variants
    python make_rodless_hinge.py keyed snap      # selected variants
Writes CoinBox/<variant>/ (STEP of both bodies, STL per part, animated GLB of the assembly
showing how it goes together and opens/closes, preview PNG).
The lid's flat top gets a debossed leather grain in the meshes (STL and GLB; the STEP stays
smooth), in two styles: `grain` and `stitched` (grain plus a stitch line around the panel).
Keyed and snap write coin_<v>_tray.stl and coin_<v>_lid_<style>.stl, print_in_place writes
coin_print_in_place_<style>.stl; each variant gets coin_<v>_<style>.glb, and
coin_<v>_leather.png previews the texture.
Original/coin.STEP is never modified. Every check prints PASS/FAIL; exit code 1 if any fail.
"""
import math
import sys
from pathlib import Path

import cadquery as cq
from cadquery import Vector as V
from OCP.BRep import BRep_Tool
from OCP.Bnd import Bnd_Box
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid, BRepBuilderAPI_MakeVertex, BRepBuilderAPI_Sewing
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepLProp import BRepLProp_CLProps, BRepLProp_SLProps
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepTools import BRepTools
from OCP.gp import gp_Pnt, gp_Vec
from OCP.TopAbs import TopAbs_EDGE, TopAbs_Orientation
from OCP.TopExp import TopExp
from OCP.TopLoc import TopLoc_Location
from OCP.TopTools import TopTools_IndexedMapOfShape
from OCP.TopoDS import TopoDS

HERE = Path(__file__).resolve().parent
SRC = HERE / "Original" / "coin.STEP"
OUT = HERE / "CoinBox"

# ---- tunables (mm) ------------------------------------------------------------
OLD_BORE_R = 1.206      # Ø2.41 rod bore in the original

PIN_D = 3.0             # keyed + snap lid pins
PIN_L = 5.0             # length out of the barrel end face
PIN_CHAMFER = 0.4
SOCKET_D = 3.3          # 0.15 radial running clearance
SOCKET_DEPTH = 5.3      # from the knuckle inner face; the rest of the knuckle stays solid
LEAD_IN = 0.8           # 45° chamfer at the slot / throat entry

FLATS_W = 2.2           # keyed: distance across the pin flats
SLOT_W = 2.3            # keyed: slot width (0.05 clearance per side over the flats)
NECK_W = 2.05           # keyed: click neck where the slot meets the socket
NECK_LEN = 0.5
KEY_ANGLE = 45.0        # keyed: lid opening angle at which the flats line up with the slot

THROAT_W = 2.5          # snap: C-socket throat (~0.83 x PIN_D)

PIP_GAP = 0.4           # print-in-place: clearance cut around barrel and knuckles
CONE_BASE_D = 4.0       # print-in-place cone pin, 45° taper
CONE_TIP_D = 0.8
CONE_GAP = 0.35         # normal clearance between cone pin and cone socket
PIP_POSE = 90.0         # print-in-place export pose (lid opening angle)

MAX_OPEN = 96           # lid first touches the tray at 97°

# The lid's underside pocket (67 x 39 x 3.7 mm) is only needed as swing clearance in a strip
# reaching 0.8 mm behind the hinge axis; the rest is filled, 45° rear face so it prints unsupported
POCKET_FILL_Z = 0.0     # fill starts this far forward of the hinge axis, at the underside
STL_TOL, STL_ANG = 0.01, 0.1
STL_WELD = 1e-4         # OCC meshes each face separately; weld so the STL is one closed shell
STL_EDGE = 1.5          # target triangle edge on planar faces (OCC leaves them as long sliver fans)
STL_SAG = 0.01          # lid: curved faces get edges of sqrt(8 * STL_SAG * radius), at least STL_EDGE_MIN
STL_EDGE_MIN = 0.2

# leather texture on the lid top: debossed only, so the lid's outer envelope (and every hinge
# clearance above) is unchanged
LEATHER = ("grain", "stitched")
TEX_CELL = 1.5          # mean pebble size
TEX_DEPTH = 0.35        # crease depth
TEX_CREASE_W = 0.45     # crease width at half depth
TEX_EDGE = 0.25         # target triangle edge on the textured face
TEX_MARGIN = 2.0        # smooth flat border inside the face edge
TEX_FADE = 1.0          # ramp from the flat border to full grain
TEX_SEED = 7
TEX_MIN_WALL = 1.5      # material left under the texture
STITCH_INSET = 3.0      # stitch line distance from the face edge
STITCH_LEN, STITCH_GAP = 2.0, 1.2
STITCH_W = 0.6
STITCH_DEPTH = 0.45

FAILS = []


def check(ok, msg):
    print(f"  [{'PASS' if ok else 'FAIL'}] {msg}")
    if not ok:
        FAILS.append(msg)
    return ok


# ---- hinge geometry, measured from the STEP ---------------------------------------
class Hinge:
    """Axis (along X through ay, az) and the hinge x-stations, read from the rod bores."""

    def __init__(self, tray, lid):
        def cyls(solid):
            out = []
            for f in solid.Faces():
                if f.geomType() == "CYLINDER":
                    c = BRepAdaptor_Surface(f.wrapped).Cylinder()
                    p, d = c.Axis().Location(), c.Axis().Direction()
                    out.append((c.Radius(), p, d, f.BoundingBox()))
            return out

        def bores(solid):
            return [c for c in cyls(solid) if abs(c[0] - OLD_BORE_R) < 0.01]

        tb, lb = bores(tray), bores(lid)
        assert tb and lb, "rod bores not found - is this the original coin.STEP?"
        self.ay, self.az = tb[0][1].Y(), tb[0][1].Z()
        self.xk = min(c[3].xmin for c in tb if c[3].xmin > 0)    # knuckle inner face
        self.xo = max(c[3].xmax for c in tb)                     # knuckle outer end of bore
        self.xb = max(c[3].xmax for c in lb)                     # lid barrel end face
        self.xli = min(c[3].xmin for c in lb if c[3].xmin > 0)   # blind end of the lid bore

        def coaxial_r(solid):
            rs = [c[0] for c in cyls(solid)
                  if abs(abs(c[2].X()) - 1) < 1e-6 and abs(c[1].Y() - self.ay) < 0.01
                  and abs(c[1].Z() - self.az) < 0.01 and 2 < c[0] < 5]
            return max(rs)

        self.rk = coaxial_r(tray)   # knuckle outer radius
        self.rb = coaxial_r(lid)    # barrel outer radius

    def __str__(self):
        return (f"axis Y={self.ay:.3f} Z={self.az:.3f} | barrel r={self.rb:.3f} end |x|={self.xb:.2f}"
                f" | knuckle r={self.rk:.3f} |x| {self.xk:.2f}..{self.xo:.2f} | lid bore from {self.xli:.2f}")


H = None  # set in main()


def xcyl(r, x0, x1):
    return cq.Solid.makeCylinder(r, x1 - x0, V(x0, H.ay, H.az), V(1, 0, 0))


def xcone(r0, r1, x0, x1):
    return cq.Solid.makeCone(r0, r1, x1 - x0, V(x0, H.ay, H.az), V(1, 0, 0))


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def yz_prism(pts, x0, x1):
    """Prism along X from a (y, z) polygon."""
    wp = cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(pts).close().extrude(x1 - x0)
    return wp.val()


def both_sides(shape):
    """Shape built for the +X hinge, plus its mirror for the -X hinge."""
    return shape.fuse(shape.mirror("YZ")).clean()


def rot(lid, angle):
    """Lid opened by `angle` degrees about the hinge axis."""
    return lid.rotate(V(0, H.ay, H.az), V(1, H.ay, H.az), -angle)


def overlap(a, b):
    return a.intersect(b).Volume()


def gap(a, b):
    """Minimum distance and where it occurs (point on `a`, rounded)."""
    d = BRepExtrema_DistShapeShape(a.wrapped, b.wrapped)
    d.Perform()
    pts = sorted({(round(abs(p.X()), 1), round(p.Y(), 1), round(p.Z(), 1))
                  for p in (d.PointOnShape1(i) for i in range(1, d.NbSolution() + 1))})
    return d.Value(), ", ".join(f"|x|={x} y={y} z={z}" for x, y, z in pts[:3])


def boolean(op, a, tool, what):
    """Fuse/cut with validity and volume-consistency checks; returns one solid.
    No .clean(): UnifySameDomain leaves the imported tray invalid."""
    if op == "fuse":
        r = a.fuse(tool)
        expect = tool.cut(a).Volume()
    else:
        r = a.cut(tool)
        expect = -a.intersect(tool).Volume()
    sols = r.Solids()
    s = sols[0] if len(sols) == 1 else r
    dv = s.Volume() - a.Volume()
    ok = len(sols) == 1 and s.isValid() and abs(dv - expect) <= max(0.02 * abs(expect), 0.05)
    check(ok, f"{what}: {op} -> {len(sols)} solid, valid={s.isValid()}, dV={dv:+.2f} (expect {expect:+.2f})")
    return s


def bore_faces(solid):
    return [f for f in solid.Faces() if f.geomType() == "CYLINDER"
            and abs(BRepAdaptor_Surface(f.wrapped).Cylinder().Radius() - OLD_BORE_R) < 0.01]


# ---- common: fill the rod bores ---------------------------------------------------
def bore_plugs(solid):
    """Exact solids filling the rod bores: the bore's own faces plus a cap over each end.
    The tray bore exits through a sloped fillet, so a flat-ended plug would leave a bump
    or a dent there; that end gets an N-sided surface spanning the exit loop instead."""
    plugs = []
    for side in (1, -1):
        faces = [f for f in bore_faces(solid) if f.Center().x * side > 0]
        ends = [e for f in faces for e in f.Edges() if e.geomType() != "LINE"]
        mid = sum(abs(e.Center().x) for e in ends) / len(ends)
        sew = BRepBuilderAPI_Sewing(1e-4)
        for f in faces:
            sew.Add(f.wrapped)
        for grp in ([e for e in ends if abs(e.Center().x) < mid], [e for e in ends if abs(e.Center().x) >= mid]):
            if all(e.geomType() == "CIRCLE" for e in grp):
                cap = cq.Face.makeFromWires(cq.Wire.assembleEdges(grp))
            else:
                cap = cq.Face.makeNSidedSurface(grp, [])
            sew.Add(cap.wrapped)
        sew.Perform()
        plug = cq.Solid(BRepBuilderAPI_MakeSolid(TopoDS.Shell_s(sew.SewedShape())).Solid())
        if plug.Volume() < 0:
            plug = cq.Solid(plug.wrapped.Reversed())
        plugs.append(plug)
    return plugs


def fill_old_bores(tray, lid):
    print("fill old rod bores")
    out = []
    for name, solid in (("tray", tray), ("lid", lid)):
        plugs = bore_plugs(solid)
        pv = sum(p.Volume() for p in plugs)
        check(all(p.isValid() for p in plugs), f"{name}: bore plugs valid ({pv:.1f} mm3)")
        for p in plugs:
            solid = boolean("fuse", solid, p, f"{name} bore plug")
        check(not bore_faces(solid), f"{name}: no Ø2.41 bore faces left")
        out.append(solid)
    return out


# ---- common: shrink the lid's underside pocket ---------------------------------------
def shrink_pocket(tray, lid):
    """Fill the pocket in the lid's underside from POCKET_FILL_Z ahead of the hinge axis to its
    front, leaving open the strip at the hinge edge that the tray's back wall swings through."""
    print("shrink lid underside pocket")
    down = sorted((f for f in lid.Faces() if f.geomType() == "PLANE" and f.normalAt().y < -0.999),
                  key=lambda f: -f.Area())
    y_under = down[0].Center().y
    roof = next(f for f in down if f.Center().y > y_under + 1)
    y_roof = roof.Center().y
    # the pocket: the roof plus the fillets running from it down to the underside
    pocket = [f for f in lid.Faces() if (b := f.BoundingBox()).ymin > y_under - 1e-3
              and b.ymax < y_roof + 1e-3 and abs(b.xmin) < 40 and abs(b.xmax) < 40 and b.zmin > H.az - 5]
    pb = cq.Compound.makeCompound(pocket).BoundingBox()
    x1, z0, z1, y1 = pb.xmax + 0.5, H.az + POCKET_FILL_Z, pb.zmax + 0.5, y_roof + 0.5
    tool = yz_prism([(y_under, z0), (y1, z0 + y1 - y_under), (y1, z1), (y_under, z1)], -x1, x1)
    fill = tool.cut(lid)
    fb = fill.BoundingBox()
    check(fb.xmax < x1 - 0.1 and fb.xmin > -x1 + 0.1 and fb.zmax < z1 - 0.1,
          f"pocket fill stays inside the pocket (|x| <= {max(fb.xmax, -fb.xmin):.2f}, z <= {fb.zmax:.2f}; "
          f"pocket |x| <= {pb.xmax:.2f}, z <= {pb.zmax:.2f})")
    lid = boolean("fuse", lid, fill, "lid + pocket fill")
    # nowhere in the swing may the fill come closer to the tray than the closed lid's underside
    top = tray.intersect(box(-x1 - 5, x1 + 5, y_under - 8, y_under + 30, H.az - 10, z1 + 5))
    g0 = gap(fill, top)[0]
    g, at = min((gap(rot(fill, a), top)[0], a) for a in range(2, MAX_OPEN + 1, 2))
    check(g >= g0 - 1e-3, f"pocket fill clears the tray through 2-{MAX_OPEN}° (min gap {g:.3f} mm at {at}°, "
                          f"closed {g0:.3f} mm); pocket left open {z0 - pb.zmin:.2f} mm from the hinge edge")
    return lid


# ---- pins and sockets (built for +X, mirrored) --------------------------------------
def round_pin():
    x0 = H.xb - 0.5  # start inside the barrel so the fuse is solid
    return (cq.Workplane("YZ", origin=(x0, 0, 0)).center(H.ay, H.az).circle(PIN_D / 2)
            .extrude(H.xb + PIN_L - x0).faces(">X").edges().chamfer(PIN_CHAMFER).val())


def keyed_pin():
    # The slab's flats run along Z; turn them so that they run along +Y (the slot) when the
    # lid is KEY_ANGLE open: opening rotates the lid by -KEY_ANGLE, so pre-rotate KEY_ANGLE - 90.
    slab = box(H.xb - 1, H.xb + PIN_L + 1, -FLATS_W / 2, FLATS_W / 2, -10, 10)
    slab = slab.rotate(V(0, 0, 0), V(1, 0, 0), KEY_ANGLE - 90).translate(V(0, H.ay, H.az))
    return round_pin().intersect(slab)


def lead_in(x0, x1, w):
    top = H.ay + H.rk
    zc = H.az
    return yz_prism([(top - LEAD_IN, zc - w / 2), (top - LEAD_IN, zc + w / 2),
                     (top + 1, zc + w / 2 + LEAD_IN + 1), (top + 1, zc - w / 2 - LEAD_IN - 1)], x0, x1)


def keyed_socket(neck_w=NECK_W):
    x0, x1 = H.xk - 0.5, H.xk + SOCKET_DEPTH
    rs = SOCKET_D / 2
    y_neck = H.ay + math.sqrt(rs ** 2 - (neck_w / 2) ** 2) + NECK_LEN
    top = H.ay + H.rk + 1
    tool = xcyl(rs, x0, x1)
    tool = tool.fuse(box(x0, x1, H.ay, y_neck, H.az - neck_w / 2, H.az + neck_w / 2))
    tool = tool.fuse(box(x0, x1, y_neck, top, H.az - SLOT_W / 2, H.az + SLOT_W / 2))
    return tool.fuse(lead_in(x0, x1, SLOT_W)).clean()


def snap_socket(throat_w=THROAT_W):
    x0, x1 = H.xk - 0.5, H.xk + SOCKET_DEPTH
    top = H.ay + H.rk + 1
    tool = xcyl(SOCKET_D / 2, x0, x1)
    tool = tool.fuse(box(x0, x1, H.ay, top, H.az - throat_w / 2, H.az + throat_w / 2))
    return tool.fuse(lead_in(x0, x1, throat_w)).clean()


def cone_pin(xb_trim):
    rb, rt = CONE_BASE_D / 2, CONE_TIP_D / 2
    x_tip = xb_trim + (rb - rt)            # 45° taper
    return xcone(rb + 0.5, rt, xb_trim - 0.5, x_tip)  # starts 0.5 inside the barrel


def cone_socket(xb_trim):
    rb, rt = CONE_BASE_D / 2, CONE_TIP_D / 2
    off = CONE_GAP * math.sqrt(2)          # normal offset of a 45° cone, measured radially
    x_tip = xb_trim + (rb - rt)
    x_end = x_tip + CONE_GAP               # clearance past the pin tip
    r_at = lambda x: rb + off - (x - xb_trim)
    x0 = H.xk - 0.5
    tool = xcone(r_at(x0), max(r_at(x_end), rt + CONE_GAP), x0, x_end)
    return tool.fuse(xcyl(rt + CONE_GAP, x0, x_end)).clean()


# ---- variants ---------------------------------------------------------------------
def swing_check(tray, lid, label):
    worst, at = 0.0, 0
    for a in range(0, MAX_OPEN + 1, 2):
        v = overlap(tray, rot(lid, a))
        if v > worst:
            worst, at = v, a
    check(worst < 0.01, f"{label}: lid swings 0-{MAX_OPEN}° without hitting the tray (worst overlap {worst:.3f} mm3 at {at}°)")


def lift(lid, angle, dy):
    return rot(lid, angle).translate(V(0, dy, 0))


def build_keyed(tray, lid):
    print("\n== keyed drop-in pins ==")
    pin = keyed_pin()
    # flats must present FLATS_W across Z when the lid is KEY_ANGLE open
    w = rot(pin, KEY_ANGLE).BoundingBox().zlen
    check(abs(w - FLATS_W) < 0.02, f"pin flats are {w:.3f} wide across the slot at {KEY_ANGLE:.0f}° open")
    lid_k = boolean("fuse", lid, both_sides(pin), "lid + keyed pins")
    tray_k = boolean("cut", tray, both_sides(keyed_socket()), "tray - keyed sockets")
    swing_check(tray_k, lid_k, "keyed")

    # assembly path: with the neck opened to the slot width the lid drops straight in at KEY_ANGLE
    tray_free = tray.cut(both_sides(keyed_socket(neck_w=SLOT_W)))
    worst = max(overlap(tray_free, lift(lid_k, KEY_ANGLE, dy / 2)) for dy in range(0, 17))
    check(worst < 0.01, f"keyed: lid lifts 0-8 mm straight up at {KEY_ANGLE:.0f}° (neck opened) - overlap {worst:.3f}")
    click = overlap(tray_k, lift(lid_k, KEY_ANGLE, 1.5))
    check(click > 0, f"keyed: neck interferes on the way in at {KEY_ANGLE:.0f}° (click) - overlap {click:.3f}")
    for a in (0, 90):
        v = overlap(tray_free, lift(lid_k, a, 0.5))
        check(v > 0.05, f"keyed: lid held at {a}° (0.5 mm lift collides, overlap {v:.3f}, neck ignored)")
    return tray_k, lid_k, KEY_ANGLE


def build_snap(tray, lid):
    print("\n== C-clip snap pins ==")
    lid_s = boolean("fuse", lid, both_sides(round_pin()), "lid + round pins")
    tray_s = boolean("cut", tray, both_sides(snap_socket()), "tray - C sockets")
    swing_check(tray_s, lid_s, "snap")
    tray_free = tray.cut(both_sides(snap_socket(throat_w=SOCKET_D)))
    worst = max(overlap(tray_free, lift(lid_s, 0, dy / 2)) for dy in range(0, 17))
    check(worst < 0.01, f"snap: with the throat opened, lid lifts 0-8 mm clear - overlap {worst:.3f}")
    for a in (0, 90):
        v = overlap(tray_s, lift(lid_s, a, 0.5))
        check(v > 0.05, f"snap: throat holds the pin at {a}° (0.5 mm lift collides, overlap {v:.3f})")
    return tray_s, lid_s, 0.0


def build_pip(tray, lid):
    print("\n== print-in-place cone pins ==")
    xb_trim = H.xk - PIP_GAP
    # lid: trim the barrel ends and open the notches around the knuckles
    lid_p = boolean("cut", lid, both_sides(xcyl(H.rk + PIP_GAP, xb_trim, H.xo + 1)), "lid clearance")
    lid_p = boolean("fuse", lid_p, both_sides(cone_pin(xb_trim)), "lid + cone pins")
    # tray: deepen the barrel seat right up to the knuckles (taking the knuckle root fillet
    # with it), then the cone sockets
    x_ch = H.xk - 0.05
    tray_p = boolean("cut", tray, xcyl(H.rb + PIP_GAP, -x_ch, x_ch), "tray barrel channel")
    tray_p = boolean("cut", tray_p, both_sides(cone_socket(xb_trim)), "tray - cone sockets")
    # The knuckle tops are flat, reaching past their cylinder radius, and sit 0.25 mm under the
    # lid in the print pose. Grow the knuckles PIP_GAP (26 shifted copies) and clear the lid.
    # (Fusing the copies fails on the B-splines, so each is cut from the lid in turn.)
    knuckles = tray_p.intersect(both_sides(box(H.xk - 0.5, H.xo + 1, H.ay, H.ay + 8, H.az - 8, H.az + 8)))
    lid_pose = before = rot(lid_p, PIP_POSE)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx or dy or dz:
                    s = PIP_GAP / math.sqrt(dx * dx + dy * dy + dz * dz)
                    lid_pose = lid_pose.cut(knuckles.translate(V(dx * s, dy * s, dz * s)))
    sols = lid_pose.Solids()
    lid_pose = sols[0]
    check(len(sols) == 1 and lid_pose.isValid(),
          f"lid clear of knuckle tops: {len(sols)} solid, valid={lid_pose.isValid()}, "
          f"dV={lid_pose.Volume() - before.Volume():+.2f}")
    lid_p = rot(lid_pose, -PIP_POSE)
    swing_check(tray_p, lid_p, "print_in_place")
    g, where = gap(tray_p, rot(lid_p, PIP_POSE))
    check(g >= 0.3, f"print_in_place: parts separate everywhere in the {PIP_POSE:.0f}° print pose "
                    f"(gap {g:.3f} mm, tightest at {where})")
    return tray_p, lid_p, PIP_POSE


# ---- output -----------------------------------------------------------------------
def face_tessellation(shape):
    """OCC tessellation welded into closed shells, with each triangle's B-rep face index and the
    unit outward normal of every planar face."""
    import numpy as np
    shape.mesh(STL_TOL, STL_ANG)
    verts, tris, fid, planes = [], [], [], {}
    for i, f in enumerate(shape.Faces()):
        # read the shape's triangulation as cadquery's tessellate() does; Face.tessellate() would
        # re-mesh faces on their own and the shared edges would no longer match
        loc = TopLoc_Location()
        poly = BRep_Tool.Triangulation_s(f.wrapped, loc)
        if poly is None:
            continue
        trsf = loc.Transformation()
        t = np.array([[tr.Value(1), tr.Value(2), tr.Value(3)] for tr in poly.Triangles()]) - 1
        if f.wrapped.Orientation() == TopAbs_Orientation.TopAbs_REVERSED:
            t = t[:, [0, 2, 1]]
        tris.append(t + len(verts))
        verts += [poly.Node(j).Transformed(trsf).Coord() for j in range(1, poly.NbNodes() + 1)]
        fid += [i] * len(t)
        if f.geomType() == "PLANE":
            planes[i] = None
    V, inv = np.unique(np.round(np.array(verts) / STL_WELD) * STL_WELD, axis=0, return_inverse=True)
    F = inv.reshape(-1)[np.concatenate(tris)]
    fid = np.array(fid)
    keep = (F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 2] != F[:, 0])
    F, fid = F[keep], fid[keep]
    for i in planes:
        tv = V[F[fid == i]]
        n = np.cross(tv[:, 1] - tv[:, 0], tv[:, 2] - tv[:, 0]).sum(0)
        planes[i] = n / np.linalg.norm(n)
    return V, F, fid, planes


def remesh_planar(V, F, fid, planes, L=STL_EDGE, passes=6):
    """Isotropic remesh of the planar faces to edge length ~L (Botsch & Kobbelt).

    OCC triangulates a planar face from its boundary nodes only: fans of slivers up to the full
    length of the lid. Viewers that average vertex normals then smear a fillet's tilt across the
    whole face, which reads as a rough, washed-out surface. Long edges are split at their
    midpoint (also where they border a curved face, which is only subdivided); collapses, flips
    and smoothing act inside planar faces and stay in their plane. The surface is unchanged,
    only its triangles are reshaped."""
    import numpy as np
    P = [np.array(p, float) for p in V]
    T = [list(map(int, t)) for t in F]
    TF = [int(i) for i in fid]
    alive = [True] * len(T)
    VT = [set() for _ in P]
    for k, t in enumerate(T):
        for v in t:
            VT[v].add(k)
    hi, lo = 4 / 3 * L, 4 / 5 * L

    def tris_of(a, b):
        return VT[a] & VT[b]

    def nbrs(v):
        return {w for k in VT[v] for w in T[k]} - {v}

    def plane_of(v):
        """Planar face index if v is interior to a planar face, else None."""
        ids = {TF[k] for k in VT[v]}
        i = ids.pop() if len(ids) == 1 else None
        return i if i in planes else None

    def area_n(a, b, c, n):
        return np.dot(np.cross(P[b] - P[a], P[c] - P[a]), n) / 2

    def elen(e):
        return np.linalg.norm(P[e[0]] - P[e[1]])

    def planar_edges():
        return {(min(t[j], t[j - 1]), max(t[j], t[j - 1]))
                for k, t in enumerate(T) if alive[k] and TF[k] in planes for j in range(3)}

    def split(a, b):
        m = len(P)
        P.append((P[a] + P[b]) / 2)
        VT.append(set())
        for k in list(tris_of(a, b)):
            t = T[k]
            j = next(j for j in range(3) if {t[j], t[(j + 1) % 3]} == {a, b})
            p, q, r = t[j], t[(j + 1) % 3], t[(j + 2) % 3]
            T[k] = [p, m, r]
            VT[q].discard(k)
            VT[m].add(k)
            T.append([m, q, r])
            TF.append(TF[k])
            alive.append(True)
            for v in (m, q, r):
                VT[v].add(len(T) - 1)

    def collapse(u, v):
        """Merge u (interior to a planar face) into v."""
        i = plane_of(u)
        shared = tris_of(u, v)
        if i is None or len(shared) != 2:
            return False
        if nbrs(u) & nbrs(v) != {w for k in shared for w in T[k]} - {u, v}:  # link condition
            return False
        for k in VT[u] - shared:
            t = [v if w == u else w for w in T[k]]
            if area_n(*t, planes[i]) < 1e-4 or max(np.linalg.norm(P[v] - P[w]) for w in t) > hi:
                return False
        for k in shared:
            alive[k] = False
            for w in T[k]:
                VT[w].discard(k)
        for k in VT[u]:
            T[k] = [v if w == u else w for w in T[k]]
            VT[v].add(k)
        VT[u] = set()
        return True

    def angle(o, x, y):
        ex, ey = P[x] - P[o], P[y] - P[o]
        return np.arctan2(np.linalg.norm(np.cross(ex, ey)), np.dot(ex, ey))

    def flip(a, b):
        """Swap edge ab of two triangles in one planar face if that makes it Delaunay."""
        ks = tris_of(a, b)
        if len(ks) != 2:
            return False
        k1, k2 = ks
        if TF[k1] != TF[k2] or TF[k1] not in planes:
            return False
        if not any(T[k1][j] == a and T[k1][(j + 1) % 3] == b for j in range(3)):
            k1, k2 = k2, k1
        c = next(w for w in T[k1] if w not in (a, b))
        d = next(w for w in T[k2] if w not in (a, b))
        n = planes[TF[k1]]
        if (tris_of(c, d) or len(VT[a]) <= 3 or len(VT[b]) <= 3
                or angle(c, a, b) + angle(d, a, b) <= math.pi + 1e-9
                or area_n(a, d, c, n) < 1e-6 or area_n(d, b, c, n) < 1e-6):
            return False
        VT[b].discard(k1)
        VT[a].discard(k2)
        T[k1], T[k2] = [a, d, c], [d, b, c]
        VT[d].add(k1)
        VT[c].add(k2)
        return True

    for _ in range(passes):
        for e in sorted(planar_edges(), key=elen, reverse=True):
            if elen(e) > hi and tris_of(*e):
                split(*e)
        for e in sorted(planar_edges(), key=elen):
            if elen(e) < lo and tris_of(*e):
                collapse(*e) or collapse(*e[::-1])
        for _ in range(20):
            if not sum(flip(*e) for e in planar_edges()):
                break
        for v in range(len(P)):  # tangential smoothing; all neighbours lie in the same plane
            i = plane_of(v) if VT[v] else None
            if i is None:
                continue
            old, n = P[v], planes[i]
            c = np.mean([P[w] for w in nbrs(v)], axis=0) - old
            P[v] = old + c - np.dot(c, n) * n
            if any(area_n(*T[k], n) < 1e-6 for k in VT[v]):
                P[v] = old

    T = np.array([t for k, t in enumerate(T) if alive[k]])
    TF = np.array([f for k, f in enumerate(TF) if alive[k]])
    used, T = np.unique(T, return_inverse=True)
    return np.array(P)[used], T.reshape(-1, 3), TF


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def _unit(a):
    n = _norm(a)
    return (a[0] / n, a[1] / n, a[2] / n) if n > 1e-14 else (0.0, 0.0, 0.0)


def surface_mesh(shape, passes=5):
    """Curvature-adaptive isotropic mesh of every face, each vertex exactly on the CAD.

    OCC meshes a face in its (u, v) parameter space; on the lid's front round (a torus with a
    1845 mm major radius) that space is so stretched that the triangles come out long and
    irregular, and averaged vertex normals show them as facets. Starting from an OCC mesh,
    edges are split, collapsed, flipped and smoothed towards a per-face target length
    (sqrt(8 * STL_SAG * radius), clamped to STL_EDGE_MIN..STL_EDGE). Each vertex keeps its (u, v)
    per triangle corner and is placed with the face's surface S(u, v); vertices on B-rep edges
    stay fixed or, when an edge is split, go to the edge curve at the mid parameter."""
    import numpy as np

    def target_len(k):
        return STL_EDGE if k < 1e-9 else min(STL_EDGE, max(STL_EDGE_MIN, math.sqrt(8 * STL_SAG / k)))

    shape = shape.copy()  # mesh a copy: the GLB and preview reuse the original's triangulation
    BRepMesh_IncrementalMesh(shape.wrapped, STL_TOL, False, STL_ANG, True)  # absolute deflection
    emap = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(shape.wrapped, TopAbs_EDGE, emap)
    curves, Le = {}, {}
    for i in range(1, emap.Extent() + 1):
        e = TopoDS.Edge_s(emap.FindKey(i))
        if BRep_Tool.Degenerated_s(e):
            continue
        c = curves[i] = BRepAdaptor_Curve(e)
        k = 0.0
        for s in np.linspace(c.FirstParameter(), c.LastParameter(), 12):
            pr = BRepLProp_CLProps(c, float(s), 2, 1e-7)
            if pr.IsTangentDefined():
                k = max(k, abs(pr.Curvature()))
        Le[i] = target_len(k)

    surfs, Lf = [], []
    P, T, TF, CU, fixed = [], [], [], [], []   # points, triangles, face index, corner (u, v)s
    FE = {}                                     # mesh edge on a B-rep edge -> (edge, t_lo, t_hi)
    key = {}

    def vid(p):
        k = tuple(np.round(np.array(p) / STL_WELD).astype(np.int64))
        if k not in key:
            key[k] = len(P)
            P.append(tuple(float(x) for x in p))
            fixed.append(False)
        return key[k]

    for fi, f in enumerate(shape.Faces()):
        sa = BRepAdaptor_Surface(f.wrapped)
        surfs.append(sa)
        u0, u1, v0, v1 = BRepTools.UVBounds_s(f.wrapped)
        k = 0.0
        for u in np.linspace(u0, u1, 9):
            for v in np.linspace(v0, v1, 9):
                pr = BRepLProp_SLProps(sa, float(u), float(v), 2, 1e-7)
                if pr.IsCurvatureDefined():
                    k = max(k, abs(pr.MaxCurvature()), abs(pr.MinCurvature()))
        Lf.append(target_len(k))
        loc = TopLoc_Location()
        poly = BRep_Tool.Triangulation_s(f.wrapped, loc)
        if poly is None:
            continue
        trsf = loc.Transformation()
        ids = [vid(poly.Node(j).Transformed(trsf).Coord()) for j in range(1, poly.NbNodes() + 1)]
        uvs = [np.array(poly.UVNode(j).Coord()) for j in range(1, poly.NbNodes() + 1)]
        rev = f.wrapped.Orientation() == TopAbs_Orientation.TopAbs_REVERSED
        for tr in poly.Triangles():
            c = [tr.Value(1) - 1, tr.Value(2) - 1, tr.Value(3) - 1]
            if rev:
                c = [c[0], c[2], c[1]]
            t = [ids[j] for j in c]
            if len(set(t)) == 3:
                T.append(t)
                TF.append(fi)
                CU.append([uvs[j].copy() for j in c])
        for e in f.Edges():
            ei = emap.FindIndex(e.wrapped)
            pot = BRep_Tool.PolygonOnTriangulation_s(e.wrapped, poly, loc)
            if pot is None:
                continue
            nodes = [ids[pot.Node(j) - 1] for j in range(1, pot.NbNodes() + 1)]
            for n in nodes:
                fixed[n] = True
            if ei not in curves or not pot.HasParameters():
                continue
            prm = [pot.Parameter(j) for j in range(1, pot.NbNodes() + 1)]
            for a, b, ta, tb in zip(nodes, nodes[1:], prm, prm[1:]):
                if a != b and (min(a, b), max(a, b)) not in FE:
                    FE[(min(a, b), max(a, b))] = (ei, ta, tb) if a < b else (ei, tb, ta)

    alive = [True] * len(T)
    VT = [set() for _ in P]
    for k, t in enumerate(T):
        for v in t:
            VT[v].add(k)

    def S(fi, uv):
        return surfs[fi].Value(float(uv[0]), float(uv[1])).Coord()

    def tris_of(a, b):
        return VT[a] & VT[b]

    def nbrs(v):
        return {w for k in VT[v] for w in T[k]} - {v}

    def tnormal(a, b, c):
        return _unit(_cross(_sub(P[b], P[a]), _sub(P[c], P[a])))

    def corner(k, v):
        return CU[k][T[k].index(v)]

    def target(a, b):
        ks = tris_of(a, b)
        L = min(Lf[TF[k]] for k in ks) if ks else STL_EDGE
        fe = FE.get((min(a, b), max(a, b)))
        return min(L, Le[fe[0]]) if fe else L

    def elen(a, b):
        return _norm(_sub(P[a], P[b]))

    def all_edges():
        return {(min(t[j], t[j - 1]), max(t[j], t[j - 1])) for k, t in enumerate(T) if alive[k] for j in range(3)}

    def split(a, b):
        e = (min(a, b), max(a, b))
        m = len(P)
        fe = FE.pop(e, None)
        ks = list(tris_of(a, b))
        if fe:
            ei, t0, t1 = fe
            tm = (t0 + t1) / 2
            P.append(curves[ei].Value(tm).Coord())
            fixed.append(True)
            FE[(min(e[0], m), max(e[0], m))] = (ei, t0, tm) if e[0] < m else (ei, tm, t0)
            FE[(min(m, e[1]), max(m, e[1]))] = (ei, tm, t1) if m < e[1] else (ei, t1, tm)
        else:
            P.append(S(TF[ks[0]], (corner(ks[0], a) + corner(ks[0], b)) / 2))
            fixed.append(False)
        VT.append(set())
        for k in ks:
            t, cu = T[k], CU[k]
            j = next(j for j in range(3) if {t[j], t[(j + 1) % 3]} == {a, b})
            p, q, r = t[j], t[(j + 1) % 3], t[(j + 2) % 3]
            up, uq, ur = cu[j], cu[(j + 1) % 3], cu[(j + 2) % 3]
            um = (up + uq) / 2
            T[k], CU[k] = [p, m, r], [up, um, ur]
            VT[q].discard(k)
            VT[m].add(k)
            T.append([m, q, r])
            CU.append([um.copy(), uq, ur])
            TF.append(TF[k])
            alive.append(True)
            for v in (m, q, r):
                VT[v].add(len(T) - 1)

    def collapse(u, v):
        """Merge u (a free vertex inside one face) into v."""
        shared = tris_of(u, v)
        if fixed[u] or len(shared) != 2:
            return False
        if nbrs(u) & nbrs(v) != {w for k in shared for w in T[k]} - {u, v}:  # link condition
            return False
        uv_v = corner(next(iter(shared)), v)
        L = target(u, v)
        for k in VT[u] - shared:
            t = [v if w == u else w for w in T[k]]
            if _dot(tnormal(*T[k]), tnormal(*t)) < 0.5 \
                    or _norm(_cross(_sub(P[t[1]], P[t[0]]), _sub(P[t[2]], P[t[0]]))) < 1e-8 \
                    or max(elen(v, w) for w in t if w != v) > 4 / 3 * L:
                return False
        for k in shared:
            alive[k] = False
            for w in T[k]:
                VT[w].discard(k)
        for k in VT[u]:
            j = T[k].index(u)
            T[k][j] = v
            CU[k][j] = uv_v.copy()
            VT[v].add(k)
        VT[u] = set()
        return True

    def angle(o, x, y):
        ex, ey = _sub(P[x], P[o]), _sub(P[y], P[o])
        return math.atan2(_norm(_cross(ex, ey)), _dot(ex, ey))

    def flip(a, b):
        """Swap edge ab inside one face if that makes it Delaunay without folding the surface."""
        ks = tris_of(a, b)
        if (min(a, b), max(a, b)) in FE or len(ks) != 2:
            return False
        k1, k2 = ks
        if TF[k1] != TF[k2]:
            return False
        if not any(T[k1][j] == a and T[k1][(j + 1) % 3] == b for j in range(3)):
            k1, k2 = k2, k1
        c = next(w for w in T[k1] if w not in (a, b))
        d = next(w for w in T[k2] if w not in (a, b))
        if tris_of(c, d) or len(VT[a]) <= 3 or len(VT[b]) <= 3 \
                or angle(c, a, b) + angle(d, a, b) <= math.pi + 1e-9:
            return False
        o1, o2 = tnormal(*T[k1]), tnormal(*T[k2])
        n1, n2 = tnormal(a, d, c), tnormal(d, b, c)
        if _dot(n1, o1) < 0.5 or _dot(n2, o2) < 0.5 or _dot(n1, n2) < min(0.9995, _dot(o1, o2)) - 1e-9:
            return False
        ua, ub, uc, ud = corner(k1, a), corner(k1, b), corner(k1, c), corner(k2, d)
        VT[b].discard(k1)
        VT[a].discard(k2)
        T[k1], CU[k1] = [a, d, c], [ua, ud.copy(), uc]
        T[k2], CU[k2] = [d, b, c], [ud, ub, uc.copy()]
        VT[d].add(k1)
        VT[c].add(k2)
        return True

    def smooth(v):
        """Move a free vertex towards its neighbours' centroid, along the surface (in u, v)."""
        k0 = next(iter(VT[v]))
        fi, uv = TF[k0], corner(k0, v)
        nb = list(nbrs(v))
        c = np.array([sum(P[w][i] for w in nb) / len(nb) - P[v][i] for i in range(3)])
        p, du, dv = gp_Pnt(), gp_Vec(), gp_Vec()
        surfs[fi].D1(float(uv[0]), float(uv[1]), p, du, dv)
        J = np.array([du.Coord(), dv.Coord()]).T
        try:
            d = np.linalg.solve(J.T @ J, J.T @ c)
        except np.linalg.LinAlgError:
            return
        before = {k: tnormal(*T[k]) for k in VT[v]}
        nuv, old = uv + 0.8 * d, P[v]
        P[v] = S(fi, nuv)
        if any(_dot(tnormal(*T[k]), n) < 0.5 for k, n in before.items()):
            P[v] = old
            return
        for k in VT[v]:
            CU[k][T[k].index(v)] = nuv.copy()

    def split_long():
        for a, b in sorted(all_edges(), key=lambda e: -elen(*e)):
            if tris_of(a, b) and elen(a, b) > 4 / 3 * target(a, b):
                split(a, b)

    def flip_all(rounds=3):
        for _ in range(rounds):
            if not sum(flip(*e) for e in all_edges()):
                break

    for _ in range(passes):
        split_long()
        for a, b in sorted(all_edges(), key=lambda e: elen(*e)):
            if VT[a] and VT[b] and tris_of(a, b) and elen(a, b) < 4 / 5 * target(a, b):
                collapse(a, b) or collapse(b, a)
        flip_all()
        for v in range(len(P)):
            if VT[v] and not fixed[v]:
                smooth(v)
    split_long()
    flip_all()

    TF = np.array([f for k, f in enumerate(TF) if alive[k]])
    T = np.array([t for k, t in enumerate(T) if alive[k]])
    used, T = np.unique(T, return_inverse=True)
    return np.array(P)[used], T.reshape(-1, 3), TF


def cad_distance(shape, pts):
    """Distance from each point to the B-rep surface of `shape`."""
    faces = cq.Compound.makeCompound(shape.Faces()).wrapped
    out = []
    for p in pts:
        d = BRepExtrema_DistShapeShape(BRepBuilderAPI_MakeVertex(gp_Pnt(*map(float, p))).Vertex(), faces)
        d.Perform()
        out.append(d.Value())
    return out


def mesh_part(shape, name, smooth):
    """Closed triangle mesh of one body: the lid fully remeshed (surface_mesh), the tray with
    OCC's mesh and its planar faces remeshed (remesh_planar; surface_mesh would take minutes)."""
    import numpy as np
    import trimesh
    if smooth:
        V, F, fid = surface_mesh(shape)
        m = trimesh.Trimesh(V, F, process=False)
        m.metadata["fid"] = fid  # B-rep face of each triangle, for leather()
        rng = np.random.default_rng(0)
        dv = max(cad_distance(shape, V[rng.choice(len(V), 150, replace=False)]))
        dc = cad_distance(shape, m.triangles_center[rng.choice(len(F), 300, replace=False)])
        e = np.linalg.norm(V[F][:, [1, 2, 0]] - V[F], axis=2)
        q = 4 * math.sqrt(3) * m.area_faces / (e ** 2).sum(1)
        check(dv < 1e-6 and max(dc) < 2 * STL_SAG and e.max() < 2 * STL_EDGE,
              f"{name}: curvature-adaptive mesh - {len(F)} triangles, median quality {np.median(q):.2f}, "
              f"longest edge {e.max():.2f} mm, vertices on the CAD surface (sampled, max {dv:.0e} mm), "
              f"chord sag max {max(dc):.3f} / mean {np.mean(dc):.4f} mm (sampled)")
        return m
    V, F, fid, planes = face_tessellation(shape)
    occ_vol = trimesh.Trimesh(V, F, process=False).volume
    V, F, fid = remesh_planar(V, F, fid, planes)
    m = trimesh.Trimesh(V, F, process=False)
    ok = m.nondegenerate_faces()
    m.update_faces(ok)
    fid = fid[ok]
    m.remove_unreferenced_vertices()
    # remeshing must not move the surface: volume as tessellated, planar vertices on their plane
    pl = np.isin(fid, list(planes))
    off = max(abs(np.dot(m.vertices[m.faces[fid == i]].reshape(-1, 3) - m.vertices[m.faces[fid == i][0, 0]],
                         n)).max() for i, n in planes.items() if np.any(fid == i))
    e = np.linalg.norm(m.vertices[m.faces][:, [1, 2, 0]] - m.vertices[m.faces], axis=2)
    q = 4 * math.sqrt(3) * m.area_faces / (e ** 2).sum(1)
    check(abs(m.volume - occ_vol) < 1e-6 * occ_vol and off < 2 * STL_WELD and e[pl].max() < 2.5 * STL_EDGE,
          f"{name}: planar faces remeshed - longest edge {e[pl].max():.2f} mm, median quality "
          f"{np.median(q[pl]):.2f}, surface unchanged (dV {m.volume - occ_vol:+.1e} mm3, "
          f"off-plane {off:.0e} mm = weld rounding)")
    return m


# ---- leather texture (mesh only) --------------------------------------------------
def lid_frame(pose):
    """Rotation matrix of rot(·, pose): closed-lid coordinates -> lid opened `pose`°,
    about the hinge axis (subtract the axis point first)."""
    import numpy as np
    c, s = math.cos(math.radians(-pose)), math.sin(math.radians(-pose))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def top_faces(shape, pose):
    """Indices of the lid's flat top face(s): planar, facing the closed lid's +Y, outermost."""
    n = V(*(lid_frame(pose) @ [0, 1, 0]))
    cand = [(i, f) for i, f in enumerate(shape.Faces())
            if f.geomType() == "PLANE" and f.normalAt(f.Center()).dot(n) > 1 - 1e-6]
    h = max(f.Center().dot(n) for _, f in cand)
    return [i for i, f in cand if abs(f.Center().dot(n) - h) < 1e-3]


def pebble_seeds():
    """Jittered-grid Voronoi seeds over the whole lid (fixed, so every variant gets the same grain)."""
    import numpy as np
    rng = np.random.default_rng(TEX_SEED)
    g = np.stack(np.meshgrid(np.arange(-95, 95, TEX_CELL), np.arange(-35, 35, TEX_CELL)), -1).reshape(-1, 2)
    return g + TEX_CELL * (0.5 + rng.uniform(-0.4, 0.4, g.shape))


def crease(xz, seeds, soft):
    """Distance to the border of the Voronoi cell each point is in: a soft minimum over the
    bisectors with the next seeds, so it has no kinks inside a cell (a hard minimum leaves
    pyramid-like ridges from the cell corners to the seed)."""
    import numpy as np
    from scipy.spatial import cKDTree
    d, i = cKDTree(seeds).query(xz, k=7)
    gap = np.linalg.norm(seeds[i[:, 1:]] - seeds[i[:, :1]], axis=2)
    e = (d[:, 1:] ** 2 - d[:, :1] ** 2) / (2 * gap)
    m = e.min(1)
    return np.clip(m - soft * np.log(np.exp(-(e - m[:, None]) / soft).sum(1)), 0, None)


def smoothstep(t):
    import numpy as np
    t = np.clip(t, 0, 1)
    return t * t * (3 - 2 * t)


def leather_depth(xz, outline, stitched):
    """Deboss depth (mm, >= 0) at closed-lid (x, z) points inside the top-face outline.
    Pebbles are Voronoi cells with rounded tops: a crease along the cell borders, its depth
    varying smoothly over the face (65-100 %), easing into a shallow shoulder; faded out
    towards the face edge. `stitched` adds rounded stitch dashes STITCH_INSET inside the edge,
    in a smooth band like a seam."""
    import numpy as np
    import shapely
    from scipy.spatial import cKDTree
    w = TEX_CREASE_W / 2 / math.sqrt(math.log(2))      # Gaussian width giving TEX_CREASE_W at half depth
    e = crease(xz, pebble_seeds(), 0.12)
    c = np.exp(-(e / w) ** 2)
    shoulder = (1 - np.clip(e / (0.45 * TEX_CELL), 0, 1)) ** 2
    rng = np.random.default_rng(TEX_SEED + 1)
    wave = sum(np.cos(xz @ (2 * math.pi / lam * np.array([math.cos(a), math.sin(a)])) + ph)
               for lam, a, ph in zip(rng.uniform(3, 9, 6), rng.uniform(0, math.pi, 6), rng.uniform(0, 2 * math.pi, 6)))
    r = 0.825 + 0.175 * np.clip(wave / 3, -1, 1)
    depth = TEX_DEPTH * (r * c + (1 - c) * 0.3 * shoulder)
    pts = shapely.points(xz)
    depth *= smoothstep((shapely.distance(outline.boundary, pts) - TEX_MARGIN) / TEX_FADE)
    if stitched:
        ring = outline.buffer(-STITCH_INSET).exterior
        depth *= smoothstep((shapely.distance(ring, pts) - STITCH_W) / STITCH_W)
        n = max(1, round(ring.length / (STITCH_LEN + STITCH_GAP)))
        pitch = ring.length / n
        s = np.concatenate([k * pitch + np.linspace(0, STITCH_LEN, int(STITCH_LEN / 0.05) + 1) for k in range(n)])
        line = np.array([(p.x, p.y) for p in shapely.line_interpolate_point(ring, s)])
        d, _ = cKDTree(line).query(xz)
        depth = np.maximum(depth, STITCH_DEPTH * np.sqrt(np.clip(1 - (2 * d / STITCH_W) ** 2, 0, 1)))
    return depth


def top_outline(m, top, pose):
    """The top face outline in closed-lid (x, z), from the mesh boundary of the top triangles."""
    import numpy as np
    from shapely import LineString
    from shapely.ops import polygonize
    Ft = m.faces[top]
    e = np.sort(np.concatenate([Ft[:, [0, 1]], Ft[:, [1, 2]], Ft[:, [2, 0]]]), axis=1)
    u, cnt = np.unique(e, axis=0, return_counts=True)
    A = np.array([0, H.ay, H.az])
    xz = ((m.vertices - A) @ lid_frame(pose) + A)[:, [0, 2]]
    return max(polygonize([LineString(xz[b]) for b in u[cnt == 1]]), key=lambda p: p.area)


def leather(m, shape, pose, stitched, label):
    """Texture the flat top of the lid mesh `m` (from mesh_part(shape, ..., smooth=True)).

    Each top-face triangle is split into k x k sub-triangles (edge ~TEX_EDGE); the non-top
    triangles along the face outline are fanned from their centroid through the new outline
    points, so the mesh stays closed. Top vertices then move inwards by leather_depth(); the
    outline lies in the flat border and stays put. Returns the textured mesh, the volume taken
    out and the outline."""
    import numpy as np
    import shapely
    import trimesh
    V0, F0, fid = m.vertices, m.faces, m.metadata["fid"]
    top = np.isin(fid, top_faces(shape, pose))
    outline = top_outline(m, top, pose)
    R, A = lid_frame(pose), np.array([0, H.ay, H.az])
    n = R @ [0, 1, 0]
    Ft = F0[top]
    # one k for all (shared edges must split alike), from the typical edge, not the odd long one
    k = math.ceil(np.percentile(np.linalg.norm(V0[Ft][:, [1, 2, 0]] - V0[Ft], axis=2), 90) / TEX_EDGE)

    # sub-triangle template over grid points (i, j), i + j <= k, at a + i/k (b - a) + j/k (c - a)
    ij = [(i, j) for i in range(k + 1) for j in range(k + 1 - i)]
    loc = {p: q for q, p in enumerate(ij)}
    tpl = []
    for i in range(k):
        for j in range(k - i):
            tpl.append((loc[i, j], loc[i + 1, j], loc[i, j + 1]))
            if i + j < k - 1:
                tpl.append((loc[i + 1, j], loc[i + 1, j + 1], loc[i, j + 1]))
    tpl = np.array(tpl)
    inner = [q for q, (i, j) in enumerate(ij) if i > 0 and j > 0 and i + j < k]
    bi = np.array([ij[q] for q in inner], float) / k

    pts, epts = [V0], {}
    count = [len(V0)]

    def add(p):
        pts.append(p)
        first = count[0]
        count[0] += len(p)
        return first

    def edge_pt(u, v, t):
        """t-th of the k - 1 points along mesh edge u -> v, shared by both triangles on it."""
        key = (u, v, t) if u < v else (v, u, k - t)
        if key not in epts:
            a, b, s = key
            epts[key] = add((V0[a] + s / k * (V0[b] - V0[a]))[None])
        return epts[key]

    a_, b_, c_ = V0[Ft[:, 0]], V0[Ft[:, 1]], V0[Ft[:, 2]]
    interior = a_[:, None] + bi[None, :, :1] * (b_ - a_)[:, None] + bi[None, :, 1:] * (c_ - a_)[:, None]
    base = add(interior.reshape(-1, 3))
    sub = []
    for t, (a, b, c) in enumerate(Ft):
        g = np.empty(len(ij), np.int64)
        g[inner] = base + t * len(inner) + np.arange(len(inner))
        for q, (i, j) in enumerate(ij):
            if i + j == 0:
                g[q] = a
            elif i == k:
                g[q] = b
            elif j == k:
                g[q] = c
            elif j == 0:
                g[q] = edge_pt(a, b, i)
            elif i == 0:
                g[q] = edge_pt(a, c, j)
            elif i + j == k:
                g[q] = edge_pt(b, c, j)
        sub.append(g[tpl])
    sub = np.concatenate(sub)

    # fan the outline neighbours through the new points on their shared edge
    split = {(u, v) for u, v, _ in epts}
    rest, fans = [], []
    for f in F0[~top]:
        if not any((min(f[j], f[j - 2]), max(f[j], f[j - 2])) in split for j in range(3)):
            rest.append(f)
            continue
        ring = []
        for j in range(3):
            p, q = f[j], f[(j + 1) % 3]
            ring.append(p)
            if (min(p, q), max(p, q)) in split:
                ring += [edge_pt(p, q, t) for t in range(1, k)]
        cen = add(V0[f].mean(0)[None])
        fans += [(cen, ring[j], ring[(j + 1) % len(ring)]) for j in range(len(ring))]
    P = np.concatenate(pts)
    F = np.concatenate([np.array(rest), np.array(fans), sub])

    # displace along -n, in the closed lid's (x, z)
    tv = np.unique(sub)
    xz = ((P[tv] - A) @ R + A)[:, [0, 2]]
    d = np.zeros(len(P))
    d[tv] = leather_depth(xz, outline, stitched)
    tri = P[sub]
    se = np.linalg.norm(tri[:, [1, 2, 0]] - tri, axis=2)
    removed = (np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1) / 2
               * d[sub].mean(1)).sum()
    smooth_top = P[tv].copy()
    P = P - d[:, None] * n
    tex = trimesh.Trimesh(P, F, process=False)

    dist = shapely.distance(outline.boundary, shapely.points(xz))
    edge = dist < TEX_MARGIN
    lim = max(TEX_DEPTH, STITCH_DEPTH if stitched else 0)
    check(tex.is_watertight and tex.is_winding_consistent and d.max() <= lim + 1e-9 and not d[tv][edge].any(),
          f"{label}: leather texture - {len(sub)} top triangles (edge mean {se.mean():.2f} / max {se.max():.2f} mm), "
          f"depth max {d.max():.2f} mm, {edge.sum()} vertices in the {TEX_MARGIN:.0f} mm border unmoved, "
          f"{removed:.0f} mm3 removed, watertight={tex.is_watertight}")
    # wall left under the texture: rays down from the smooth top through the smooth lid
    rng = np.random.default_rng(0)
    deep = np.flatnonzero(d[tv] > 0)
    near = deep[dist[deep] < TEX_MARGIN + TEX_FADE + 1]
    pick = np.concatenate([rng.choice(deep, min(3000, len(deep)), replace=False),
                           rng.choice(near, min(2000, len(near)), replace=False)])
    o = smooth_top[pick] - 1e-3 * n
    loc_, ray, _ = m.ray.intersects_location(o, np.tile(-n, (len(o), 1)), multiple_hits=True)
    thick = np.full(len(o), np.inf)
    np.minimum.at(thick, ray, np.linalg.norm(loc_ - o[ray], axis=1) + 1e-3)
    wall = thick - d[tv][pick]
    i = int(np.argmin(wall))
    x, z = xz[pick[i]]
    check(wall.min() >= TEX_MIN_WALL,
          f"{label}: wall under the texture >= {TEX_MIN_WALL} mm (min {wall.min():.2f} mm at x={x:.1f} z={z:.1f}, "
          f"{len(o)} rays)")
    return tex, removed, outline


def write_stl(path, parts):
    """One STL from [(mesh, expected volume)], one closed shell per part."""
    import trimesh
    m = trimesh.util.concatenate([p[0] for p in parts])
    m.export(path)
    shells = len(m.split(only_watertight=False))
    vol = sum(p[1] for p in parts)
    dv = abs(m.volume - vol) / vol
    check(m.is_watertight and m.is_winding_consistent and shells == len(parts) and dv < 0.005,
          f"{path.name}: watertight={m.is_watertight}, {shells} shell(s), {len(m.faces)} triangles, "
          f"volume off by {dv:.2%}")


# ---- animated GLB -----------------------------------------------------------------
# Keyframes (time s, lid opening °, lid lift mm along +Y), eased between keys.
# The first animation in each file shows how that variant goes together, then cycles the lid.
OPEN_CLOSE = [(0, 0, 0), (0.6, 0, 0), (2.2, MAX_OPEN, 0), (2.8, MAX_OPEN, 0), (4.4, 0, 0)]
TIMELINES = {
    "keyed": [  # lowered onto the knuckles at the key angle, then closed, opened, closed
        ("drop in at 45° + open/close",
         [(0, KEY_ANGLE, 10), (0.6, KEY_ANGLE, 10), (2.2, KEY_ANGLE, 0), (2.8, KEY_ANGLE, 0),
          (4.0, 0, 0), (4.6, 0, 0), (6.2, MAX_OPEN, 0), (6.8, MAX_OPEN, 0), (8.4, 0, 0), (9.0, 0, 0)]),
        ("open/close", OPEN_CLOSE)],
    "snap": [  # pressed down through the throats, then opened and closed
        ("snap on + open/close",
         [(0, 0, 10), (0.6, 0, 10), (2.2, 0, 0), (2.8, 0, 0),
          (4.4, MAX_OPEN, 0), (5.0, MAX_OPEN, 0), (6.6, 0, 0), (7.2, 0, 0)]),
        ("open/close", OPEN_CLOSE)],
    "print_in_place": [  # starts in the print pose, as it comes off the bed
        ("print pose + open/close",
         [(0, PIP_POSE, 0), (1.0, PIP_POSE, 0), (2.4, 0, 0), (3.0, 0, 0),
          (4.6, MAX_OPEN, 0), (5.2, MAX_OPEN, 0), (6.0, PIP_POSE, 0)]),
        ("open/close", OPEN_CLOSE)],
}
GLB_TOL, GLB_ANG = 0.02, 0.15
GLB_SHARP = 60          # textured lid: shade smoothly across edges flatter than this (the grain's
                        # creases stay below; the stitch walls and the body's edges stay crisp)
GLB_FPS = 30


def sample(keys):
    """Per-frame (t, angle, lift) with cosine easing between keyframes."""
    import numpy as np
    t_end = keys[-1][0]
    ts = np.linspace(0, t_end, int(round(t_end * GLB_FPS)) + 1)
    out = []
    for t in ts:
        for (t0, a0, l0), (t1, a1, l1) in zip(keys, keys[1:]):
            if t0 <= t <= t1:
                u = 0.5 - 0.5 * math.cos(math.pi * (t - t0) / (t1 - t0))
                out.append((t, a0 + (a1 - a0) * u, l0 + (l1 - l0) * u))
                break
    return np.array(out)


def lid_quat(angle):
    """glTF [x, y, z, w] for opening `angle`° (a -angle rotation about +X, as in rot())."""
    h = math.radians(-angle) / 2
    return [math.sin(h), 0.0, 0.0, math.cos(h)]


def write_glb(path, tray, lid, timelines, lid_mesh):
    """Tray mesh + lid mesh under a pivot node on the hinge axis; the animations rotate and
    lift that pivot. Units are metres, +Y up, as glTF expects (the model is already Y-up).
    The tray is tessellated from the CAD; the lid is `lid_mesh`, the textured closed-lid mesh
    (`lid`, its CAD body, is the reference for check_glb)."""
    import json
    import struct
    import numpy as np
    import trimesh

    buf = bytearray()
    views, accessors = [], []

    def add(arr, atype, ctype, target=None, minmax=False):
        while len(buf) % 4:
            buf.append(0)
        data = arr.tobytes()
        views.append({"buffer": 0, "byteOffset": len(buf), "byteLength": len(data)})
        if target:
            views[-1]["target"] = target
        buf.extend(data)
        acc = {"bufferView": len(views) - 1, "componentType": ctype, "count": len(arr), "type": atype}
        if minmax:
            a2 = arr.reshape(len(arr), -1)
            acc["min"], acc["max"] = a2.min(0).tolist(), a2.max(0).tolist()
        accessors.append(acc)
        return len(accessors) - 1

    def mesh(shape, origin):
        if isinstance(shape, trimesh.Trimesh):
            # one welded mesh: unweld it at sharp edges, or averaged normals would round them
            t = trimesh.graph.smooth_shade(shape, angle=math.radians(GLB_SHARP))
            v, f, n = (t.vertices - origin) / 1000.0, t.faces.astype(np.uint32), t.vertex_normals
        else:
            # OCC tessellates face by face without sharing vertices, so averaged normals stay
            # smooth inside each CAD face and edges between faces stay sharp
            verts, tris = shape.tessellate(GLB_TOL, GLB_ANG)
            v = (np.array([p.toTuple() for p in verts]) - origin) / 1000.0
            f = np.array(tris, dtype=np.uint32)
            n = trimesh.Trimesh(v, f, process=False).vertex_normals
        return {"attributes": {"POSITION": add(v.astype(np.float32), "VEC3", 5126, 34962, True),
                               "NORMAL": add(n.astype(np.float32), "VEC3", 5126, 34962)},
                "indices": add(f.reshape(-1), "SCALAR", 5125, 34963)}

    axis = np.array([0.0, H.ay, H.az])
    gltf = {
        "asset": {"version": "2.0", "generator": "make_rodless_hinge.py"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [
            {"name": path.stem, "children": [1, 2]},
            {"name": "tray", "mesh": 0},
            {"name": "lid_hinge", "translation": (axis / 1000).tolist(), "children": [3]},
            {"name": "lid", "mesh": 1},
        ],
        "materials": [
            {"name": "tray", "pbrMetallicRoughness": {"baseColorFactor": [*TRAY_RGB, 1],
                                                      "metallicFactor": 0, "roughnessFactor": 0.55}},
            {"name": "lid", "pbrMetallicRoughness": {"baseColorFactor": [*LID_RGB, 1],
                                                     "metallicFactor": 0, "roughnessFactor": 0.5}},
        ],
        "meshes": [{"name": "tray", "primitives": [{**mesh(tray, np.zeros(3)), "material": 0}]},
                   {"name": "lid", "primitives": [{**mesh(lid_mesh, axis), "material": 1}]}],
        "animations": [],
    }
    for anim_name, keys in timelines:
        fr = sample(keys)
        t = add(fr[:, 0].astype(np.float32), "SCALAR", 5126, minmax=True)
        q = add(np.array([lid_quat(a) for a in fr[:, 1]], dtype=np.float32), "VEC4", 5126)
        tr = add(np.array([(axis + [0, lift, 0]) / 1000 for lift in fr[:, 2]], dtype=np.float32), "VEC3", 5126)
        gltf["animations"].append({
            "name": anim_name,
            "samplers": [{"input": t, "output": q, "interpolation": "LINEAR"},
                         {"input": t, "output": tr, "interpolation": "LINEAR"}],
            "channels": [{"sampler": 0, "target": {"node": 2, "path": "rotation"}},
                         {"sampler": 1, "target": {"node": 2, "path": "translation"}}]})
    while len(buf) % 4:
        buf.append(0)
    gltf["bufferViews"], gltf["accessors"], gltf["buffers"] = views, accessors, [{"byteLength": len(buf)}]
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * (-len(js) % 4)
    with open(path, "wb") as fh:
        fh.write(struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(buf)))
        fh.write(struct.pack("<II", len(js), 0x4E4F534A) + js)
        fh.write(struct.pack("<II", len(buf), 0x004E4942) + bytes(buf))
    check_glb(path, lid, timelines)


def check_glb(path, lid, timelines):
    """Re-read the GLB, pose its lid at a few frames from the stored animation data and
    compare with the CAD lid posed the same way."""
    import json
    import struct
    import numpy as np

    raw = path.read_bytes()
    magic, _, total = struct.unpack_from("<III", raw)
    jlen = struct.unpack_from("<I", raw, 12)[0]
    g = json.loads(raw[20:20 + jlen])
    blob = raw[20 + jlen + 8:]

    def acc(i):
        a = g["accessors"][i]
        v = g["bufferViews"][a["bufferView"]]
        n = {"SCALAR": 1, "VEC3": 3, "VEC4": 4}[a["type"]]
        dt = np.float32 if a["componentType"] == 5126 else np.uint32
        return np.frombuffer(blob, dt, a["count"] * n, v["byteOffset"]).reshape(a["count"], n)

    lid_v = acc(g["meshes"][1]["primitives"][0]["attributes"]["POSITION"]).astype(float)
    worst = 0.0
    for anim, (_, keys) in zip(g["animations"], timelines):
        ts, qs = acc(anim["samplers"][0]["input"])[:, 0], acc(anim["samplers"][0]["output"])
        trs = acc(anim["samplers"][1]["output"])
        for i in np.linspace(0, len(ts) - 1, 7).astype(int):
            x, _, _, w = qs[i]
            ang = -math.degrees(2 * math.atan2(x, w))
            c, s = math.cos(math.radians(-ang)), math.sin(math.radians(-ang))
            R = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
            posed = (lid_v @ R.T + trs[i]) * 1000
            lift = trs[i][1] * 1000 - H.ay
            # the exact bounding box of the posed CAD lid (BoundingBox() is loose by ~0.15 mm, and
            # a coarse tessellation cuts the tilted front round by as much)
            b = Bnd_Box()
            BRepBndLib.AddOptimal_s(rot(lid, ang).translate(V(0, lift, 0)).wrapped, b, False, False)
            ref = np.array(b.Get()).reshape(2, 3)
            worst = max(worst, np.abs(np.array([posed.min(0), posed.max(0)]) - ref).max())
    ok = magic == 0x46546C67 and total == len(raw) and worst < 0.02
    check(ok, f"{path.name}: valid GLB container, {len(g['animations'])} animations "
              f"({', '.join(a['name'] for a in g['animations'])}), lid pose matches CAD within {worst:.3f} mm, "
              f"{len(raw) / 1e6:.1f} MB")


def export(name, tray, lid, pose):
    import numpy as np
    import trimesh
    d = OUT / name
    d.mkdir(parents=True, exist_ok=True)
    lid_pose = rot(lid, pose) if name == "print_in_place" else lid
    asm = cq.Assembly(name=f"coin_{name}")
    asm.add(tray, name="tray", color=cq.Color(0.55, 0.6, 0.65))
    asm.add(lid_pose, name="lid", color=cq.Color(0.85, 0.55, 0.25))
    asm.export(str(d / f"coin_{name}.step"))
    pip = name == "print_in_place"
    tray_m = mesh_part(tray, "tray", False)
    if not pip:
        write_stl(d / f"coin_{name}_tray.stl", [(tray_m, tray.Volume())])
    lid_m = mesh_part(lid_pose, "lid", True)
    for style in LEATHER:
        tex, removed, outline = leather(lid_m, lid_pose, pose if pip else 0, style == "stitched", f"lid {style}")
        lid_part = [(tex, lid_pose.Volume() - removed)]
        if pip:
            write_stl(d / f"coin_{name}_{style}.stl", [(tray_m, tray.Volume())] + lid_part)
            # the GLB animates the lid from closed: turn the print-pose mesh back
            A = np.array([0, H.ay, H.az])
            tex = trimesh.Trimesh((tex.vertices - A) @ lid_frame(pose) + A, tex.faces, process=False)
        else:
            write_stl(d / f"coin_{name}_lid_{style}.stl", lid_part)
        write_glb(d / f"coin_{name}_{style}.glb", tray, lid, TIMELINES[name], tex)
    render(d / f"coin_{name}_hinge.png", tray, lid, name, pose)
    render_leather(d / f"coin_{name}_leather.png", outline, name)


def render_leather(path, outline, name):
    """Shaded relief of the leather depth map on the lid top: whole face and a close-up of the
    front corner, grain and stitched."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    import shapely

    def relief(x0, x1, z0, z1, res, stitched):
        xs, zs = np.arange(x0, x1, res), np.arange(z0, z1, res)
        X, Z = np.meshgrid(xs, zs)
        inside = shapely.contains_xy(outline, X, Z)
        h = np.zeros(X.shape)
        h[inside] = -leather_depth(np.c_[X[inside], Z[inside]], outline, stitched)
        gz, gx = np.gradient(h, res)
        n = np.dstack([-gx, -gz, np.ones_like(h)])
        n /= np.linalg.norm(n, axis=2, keepdims=True)
        shade = 0.25 + 0.75 * np.clip(n @ (np.array([-0.5, 0.6, 0.62]) / np.linalg.norm([-0.5, 0.6, 0.62])), 0, 1)
        rgb = np.outer(shade, LEATHER_RGB).reshape(*h.shape, 3).clip(0, 1)
        rgb[~inside] = 1
        return rgb, (x0, x1, z0, z1)

    b = outline.bounds
    fig = plt.figure(figsize=(16, 9))
    for row, stitched in enumerate((False, True)):
        style = LEATHER[row]
        ax = fig.add_subplot(2, 3, (3 * row + 1, 3 * row + 2))
        img, ext = relief(b[0], b[2], b[1], b[3], 0.08, stitched)
        ax.imshow(img, origin="lower", extent=ext)
        ax.set_title(f"{style}: whole top face ({b[2] - b[0]:.0f} x {b[3] - b[1]:.0f} mm, front edge at top)")
        ax.set_axis_off()
        ax = fig.add_subplot(2, 3, 3 * row + 3)
        z1 = outline.intersection(shapely.box(b[2] - 20, b[1], b[2], b[3])).bounds[3]
        img, ext = relief(b[2] - 20, b[2] + 0.5, z1 - 14, z1 + 0.5, 0.03, stitched)
        ax.imshow(img, origin="lower", extent=ext)
        ax.set_title(f"{style}: front +X corner, 20 x 14 mm")
        ax.set_axis_off()
    fig.suptitle(f"{name}: lid leather texture - pebble {TEX_CELL} mm, crease {TEX_DEPTH} mm deep, "
                 f"{TEX_MARGIN:.0f} mm smooth border, stitches {STITCH_LEN}/{STITCH_GAP} mm")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print(f"  wrote {path.relative_to(HERE)}")


TRAY_RGB, LID_RGB = (0.62, 0.68, 0.74), (0.90, 0.58, 0.28)
LEATHER_RGB = (0.55, 0.42, 0.33)


def render(path, tray, lid, name, pose):
    """+X hinge: assembled at `pose`, the tray knuckle alone, the lid pin alone."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    import numpy as np

    light = np.array([0.4, 0.45, 0.8]) / np.linalg.norm([0.4, 0.45, 0.8])  # plot axes (x, z, y)
    near = box(H.xk - 12, H.xo + 4, H.ay - 9, H.ay + 14, H.az - 9, H.az + 14)
    knuckle = box(H.xk - 6, H.xo + 3, H.ay - 6, H.ay + 6, H.az - 7, H.az + 7)
    pin = box(H.xb - 7, H.xo, H.ay - 6, H.ay + 6, H.az - 6, H.az + 8)
    panels = [
        ("assembled", near, [(tray, TRAY_RGB), (rot(lid, pose), LID_RGB)], 25, -55),
        ("tray knuckle socket", knuckle, [(tray, TRAY_RGB)], 40, -140),
        ("lid pin (lid closed)", pin, [(lid, LID_RGB)], 20, -30),
    ]
    fig = plt.figure(figsize=(16, 6))
    for i, (title, crop, parts, elev, azim) in enumerate(panels):
        ax = fig.add_subplot(1, 3, i + 1, projection="3d")
        all_polys, all_cols = [], []
        for shape, rgb in parts:
            verts, tris = shape.intersect(crop).tessellate(0.02, 0.1)
            if not tris:
                continue
            pts = np.array([[v.x, v.z, v.y] for v in verts])  # plot with model Y as up
            polys = pts[np.array(tris)]
            n = np.cross(polys[:, 1] - polys[:, 0], polys[:, 2] - polys[:, 0])
            n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
            shade = 0.35 + 0.65 * np.abs(n @ light)
            all_polys.append(polys)
            all_cols.append(np.outer(shade, rgb).clip(0, 1))
        # one collection, so matplotlib depth-sorts the two parts against each other
        ax.add_collection3d(Poly3DCollection(np.concatenate(all_polys), facecolors=np.concatenate(all_cols),
                                             edgecolor="none"))
        b = crop.BoundingBox()
        ax.set_xlim(b.xmin, b.xmax); ax.set_ylim(b.zmin, b.zmax); ax.set_zlim(b.ymin, b.ymax)
        ax.set_box_aspect((b.xlen, b.zlen, b.ylen))
        ax.view_init(elev, azim)
        ax.set_axis_off()
        ax.set_title(title)
    fig.suptitle(f"{name}: +X hinge, assembled view with lid at {pose:.0f}° (tray grey, lid orange)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print(f"  wrote {path.relative_to(HERE)}")


BUILDERS = {"keyed": build_keyed, "snap": build_snap, "print_in_place": build_pip}


def main(argv):
    global H
    names = argv or list(BUILDERS)
    bad = [n for n in names if n not in BUILDERS]
    if bad:
        sys.exit(f"unknown variant(s) {bad}; choose from {list(BUILDERS)}")
    solids = sorted(cq.importers.importStep(str(SRC)).solids().vals(), key=lambda s: -s.Volume())
    tray, lid = solids
    H = Hinge(tray, lid)
    print(H)
    tray, lid = fill_old_bores(tray, lid)
    lid = shrink_pocket(tray, lid)
    for n in names:
        t, l, pose = BUILDERS[n](tray, lid)
        export(n, t, l, pose)
    print(f"\n{'ALL CHECKS PASSED' if not FAILS else f'{len(FAILS)} CHECK(S) FAILED:'}")
    for f in FAILS:
        print("  -", f)
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
