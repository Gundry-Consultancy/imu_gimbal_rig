"""Pan / tilt / roll gimbal around the sensor carrier. Run with freecadcmd.

    freecadcmd -c "exec(open('make_gimbal.py').read(), {'__file__': 'make_gimbal.py', '__name__': '__main__'})"

Global frame: origin is the sensor IC (axis intersection), +X is the roll axis,
+Y is the tilt axis, +Z is the pan axis (up).

Parts (exported to parts/):
  roll_cradle                   carrier sits in it; MG90S horn on +X wall, 623 idler axle on -X
  tilt_ring_mg90s_tab16 / tab21 holds the roll MG90S (two tab-height variants) and the -X 623 bearing;
                                standard-servo horn on +Y bar, idler axle on -Y
  pan_yoke                      holds the standard tilt servo (+Y) and the -Y 623 bearing;
                                pan horn on the underside
  base                          holds the standard pan servo

The servo, bearing and board solids are reference envelopes for clearance
checks. They are not printed.
"""

import math
import os
import sys

import FreeCAD as App
import Part
import Mesh

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import make_carrier as mc  # noqa: E402

V = App.Vector
PARTS_DIR = os.path.join(HERE, "parts")

# --- servos ---------------------------------------------------------------
# spline_above_tab: spline top above the tab top face (what sets mount position).
# offset: shaft centre from body centre along the body's long axis.
MG90S = dict(L=22.8, W=12.2, H=28.5, case_top=4.0, tab_len=32.2, tab_t=2.5,
             holes=(27.5, 28.0), hole_across=0.0, pilot=1.7, offset=5.5, spline_r=2.4,
             horn=dict(len=36.0, width=7.0, depth=2.0, hub=2.5, centre=6.0))
STANDARD = dict(L=40.7, W=19.9, H=42.9, case_top=4.5, tab_len=54.5, tab_t=3.0,
                holes=(48.5, 49.5), hole_across=10.0, pilot=2.4, offset=10.35, spline_r=3.0,
                spline_above_tab=12.1,
                horn=dict(len=46.0, width=8.5, depth=2.5, hub=3.5, centre=7.0))


def mg90s(tab_height):
    s = dict(MG90S)
    s["spline_above_tab"] = s["H"] - tab_height - s["tab_t"]
    s["tab_height"] = tab_height
    return s


# --- bearing 623ZZ (3 x 10 x 4) -------------------------------------------
BRG_OD, BRG_W = 10.0, 4.0
BRG_POCKET = 10.15
LIP_HOLE = 6.5
AXLE_BOSS_D = 4.8   # touches inner race only
M3_PILOT = 2.8

# --- layout ------------------------------------------------------------------
CARRIER_IC_Z = mc.THICKNESS + mc.BOSS_H + 1.6 + 0.8   # IC above carrier underside
PLATE_W, PLATE_H = 60.96, 30.48
CR_FLOOR_T = 3.0
CR_WALL_T = 4.0
CR_WALL_HALF_Y = 19.0
CR_GAP = 0.5
CR_WALL_IN = PLATE_W / 2 + CR_GAP           # inner face |x|
CR_WALL_OUT = CR_WALL_IN + CR_WALL_T        # outer face |x|
CR_Z0 = -CARRIER_IC_Z - CR_FLOOR_T          # cradle underside
CR_HUB_R = 11.0

RING_IN_Y = 27.5         # clears cradle sweep radius (~25.8)
RING_BAR_T = 6.0
RING_OUT_Y = RING_IN_Y + RING_BAR_T
RING_BAR_HALF_Z = 7.0
RING_BRG_PLATE_T = 6.0   # 4 mm bearing pocket + 2 mm lip
RING_GAP = 1.5

YOKE_PLATE_T = 6.0
YOKE_HALF_X = 16.0
YOKE_GAP = 1.5
TILT_CLEAR_R = 72.0      # yoke floor below the tilt sweep
YOKE_FLOOR_T = 6.0

SERVO_PLATE_T = 5.0


# --- helpers ---------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl(r, p0, p1):
    d = p1 - p0
    return Part.makeCylinder(r, d.Length, p0, d)


def placed(shape, origin, xdir, zdir):
    """Map a local shape (x = servo long axis / horn arm, z = shaft direction) into the world."""
    xdir, zdir = V(xdir).normalize(), V(zdir).normalize()
    ydir = zdir.cross(xdir)
    m = App.Matrix(xdir.x, ydir.x, zdir.x, origin.x,
                   xdir.y, ydir.y, zdir.y, origin.y,
                   xdir.z, ydir.z, zdir.z, origin.z,
                   0, 0, 0, 1)
    s = shape.copy()
    s.transformShape(m)
    return s


def servo_envelope(s):
    """Local frame: spline top at origin, shaft along +z, long axis along x (body centre at -offset)."""
    cx = -s["offset"]
    body = box(cx - s["L"] / 2, cx + s["L"] / 2, -s["W"] / 2, s["W"] / 2, -s["H"], -s["case_top"])
    tz1 = -s["spline_above_tab"]
    tabs = box(cx - s["tab_len"] / 2, cx + s["tab_len"] / 2, -s["W"] / 2, s["W"] / 2, tz1 - s["tab_t"], tz1)
    spline = Part.makeCylinder(s["spline_r"], s["case_top"], V(0, 0, -s["case_top"]))
    return body.fuse([tabs, spline]).removeSplitter()


def servo_plate(s, x_ext, y_ext, thickness=SERVO_PLATE_T):
    """Mount plate in servo-local frame, tabs resting on its +z face. Returns (solid, cutters)."""
    z1 = -s["spline_above_tab"] - s["tab_t"]
    z0 = z1 - thickness
    plate = box(x_ext[0], x_ext[1], y_ext[0], y_ext[1], z0, z1)
    cx = -s["offset"]
    cut = [box(cx - s["L"] / 2 - 0.25, cx + s["L"] / 2 + 0.25, -s["W"] / 2 - 0.25, s["W"] / 2 + 0.25, z0 - 1, z1 + 1)]
    h0, h1 = s["holes"]
    ys = [0.0] if s["hole_across"] == 0 else [-s["hole_across"] / 2, s["hole_across"] / 2]
    for side in (-1, 1):
        for y in ys:
            a = V(cx + side * h0 / 2, y, z0 - 1)
            b = V(cx + side * h1 / 2, y, z0 - 1)
            r = s["pilot"] / 2
            cut.append(Part.makeCylinder(r, thickness + 2, a))
            cut.append(Part.makeCylinder(r, thickness + 2, b))
            if (b - a).Length > 0:
                cut.append(box(min(a.x, b.x), max(a.x, b.x), y - r, y + r, z0 - 1, z1 + 1))
    return plate, cut


def horn_pocket(horn, through=12.0):
    """Local frame: mounting face at z=0, part material at z<0, servo at z>0. Arm along x."""
    arm = box(-horn["len"] / 2, horn["len"] / 2, -horn["width"] / 2, horn["width"] / 2, -horn["depth"], 1)
    hub = Part.makeCylinder(horn["width"] / 2 + 1.5, horn["depth"] + 1, V(0, 0, -horn["depth"]))
    centre = Part.makeCylinder(horn["centre"] / 2, through + 1, V(0, 0, -through))
    return arm.fuse([hub, centre])


def bearing_housing_cut(face_out, axis):
    """Cut for a 623 pocket from an outer face plus a lip hole. face_out on the axis, axis points outward."""
    a = V(axis).normalize()
    pocket = cyl(BRG_POCKET / 2, face_out - a * BRG_W, face_out + a * 1)
    lip = cyl(LIP_HOLE / 2, face_out - a * (RING_BRG_PLATE_T + 1), face_out)
    return pocket.fuse(lip)


def bearing_envelope(face_out, axis):
    a = V(axis).normalize()
    outer = cyl(BRG_OD / 2, face_out - a * BRG_W, face_out)
    return outer.cut(cyl(1.5, face_out - a * (BRG_W + 1), face_out + a))


def axle_boss(face, axis, length, pilot_depth):
    """Boss on a rotating part reaching the bearing inner race; axis points from part toward bearing."""
    a = V(axis).normalize()
    boss = cyl(AXLE_BOSS_D / 2, face - a * 0.5, face + a * length)
    pilot = cyl(M3_PILOT / 2, face + a * (length - pilot_depth), face + a * (length + 1))
    return boss, pilot


# --- parts -----------------------------------------------------------------
def roll_cradle():
    floor = box(-CR_WALL_OUT, CR_WALL_OUT, -PLATE_H / 2, PLATE_H / 2, CR_Z0, CR_Z0 + CR_FLOOR_T)
    walls = []
    for sx in (-1, 1):
        x0, x1 = sorted((sx * CR_WALL_IN, sx * CR_WALL_OUT))
        w = box(x0, x1, -CR_WALL_HALF_Y, CR_WALL_HALF_Y, CR_Z0, 0)
        hub = cyl(CR_HUB_R, V(x0, 0, 0), V(x1, 0, 0))
        walls.append(w.fuse(hub))
    shape = floor.fuse(walls)
    # carrier screws through the four centre-cell round holes (clear of Feather bosses)
    pilots = [Part.makeCylinder(1.1, CR_FLOOR_T + 1, V(x, y, CR_Z0 - 0.5))
              for x in (-7.62, 7.62) for y in (-7.62, 7.62)]
    # +X: MG90S horn
    horn = placed(horn_pocket(MG90S["horn"], through=CR_WALL_T + 2), V(CR_WALL_OUT, 0, 0), (0, 1, 0), (1, 0, 0))
    # -X: axle boss into the ring's bearing
    boss_len = RING_GAP + (RING_BRG_PLATE_T - BRG_W)
    boss, pilot = axle_boss(V(-CR_WALL_OUT, 0, 0), (-1, 0, 0), boss_len, boss_len + CR_WALL_T - 0.5)
    shape = shape.fuse(boss).cut(Part.makeCompound(pilots + [horn, pilot]))
    return shape.removeSplitter()


def tilt_ring(servo):
    roll_spline_x = CR_WALL_OUT + servo["horn"]["hub"]
    s_origin, s_x, s_z = V(roll_spline_x, 0, 0), (0, 0, 1), (-1, 0, 0)
    # servo plate in servo-local coords: local x = world z, local y = world y, local z = -world x
    tab_z = servo["holes"][1] / 2 + 3
    plate, plate_cuts = servo_plate(servo, (-servo["offset"] - tab_z, -servo["offset"] + tab_z),
                                    (-RING_OUT_Y, RING_OUT_Y))
    plate = placed(plate, s_origin, s_x, s_z)
    plate_cuts = [placed(c, s_origin, s_x, s_z) for c in plate_cuts]
    plate_x_out = plate.BoundBox.XMax

    brg_in = -(CR_WALL_OUT + RING_GAP)
    brg_out = brg_in - RING_BRG_PLATE_T
    brg_plate = box(brg_out, brg_in, -RING_OUT_Y, RING_OUT_Y, -RING_BAR_HALF_Z - 4, RING_BAR_HALF_Z + 4)

    bars = [box(brg_out, plate_x_out, y0, y1, -RING_BAR_HALF_Z, RING_BAR_HALF_Z)
            for y0, y1 in ((-RING_OUT_Y, -RING_IN_Y), (RING_IN_Y, RING_OUT_Y))]
    shape = plate.fuse([brg_plate] + bars)

    cuts = plate_cuts + [bearing_housing_cut(V(brg_out, 0, 0), (-1, 0, 0))]
    # +Y: tilt servo horn (arm along X)
    cuts.append(placed(horn_pocket(STANDARD["horn"], through=RING_BAR_T + 2), V(0, RING_OUT_Y, 0), (1, 0, 0), (0, 1, 0)))
    # -Y: axle boss into the yoke bearing
    boss_len = YOKE_GAP + (YOKE_PLATE_T - BRG_W)
    boss, pilot = axle_boss(V(0, -RING_OUT_Y, 0), (0, -1, 0), boss_len, boss_len + RING_BAR_T - 0.5)
    shape = shape.fuse(boss).cut(Part.makeCompound(cuts + [pilot]))
    servo_env = placed(servo_envelope(servo), s_origin, s_x, s_z)
    brg_env = bearing_envelope(V(brg_out, 0, 0), (-1, 0, 0))
    return shape.removeSplitter(), servo_env, brg_env


def pan_yoke():
    s = STANDARD
    floor_top = -TILT_CLEAR_R
    floor_bot = floor_top - YOKE_FLOOR_T
    # tilt servo on +Y
    t_origin = V(0, RING_OUT_Y + s["horn"]["hub"], 0)
    t_x, t_z = (0, 0, 1), (0, -1, 0)
    tab_half = s["tab_len"] / 2 + 3
    plate, plate_cuts = servo_plate(s, (-s["offset"] - tab_half, -s["offset"] + tab_half),
                                    (-YOKE_HALF_X, YOKE_HALF_X), thickness=YOKE_PLATE_T)
    plate = placed(plate, t_origin, t_x, t_z)
    # extend the servo plate down to the floor
    pb = plate.BoundBox
    servo_upright = box(-YOKE_HALF_X, YOKE_HALF_X, pb.YMin, pb.YMax, floor_bot, pb.ZMax)
    plate_cuts = [placed(c, t_origin, t_x, t_z) for c in plate_cuts]

    brg_in = -(RING_OUT_Y + YOKE_GAP)
    brg_out = brg_in - YOKE_PLATE_T
    idler_upright = box(-YOKE_HALF_X, YOKE_HALF_X, brg_out, brg_in, floor_bot, 12)
    idler_upright = idler_upright.fuse(cyl(YOKE_HALF_X, V(0, brg_out, 12 - YOKE_HALF_X), V(0, brg_in, 12 - YOKE_HALF_X)))
    floor = box(-YOKE_HALF_X, YOKE_HALF_X, brg_out, pb.YMax, floor_bot, floor_top)

    shape = floor.fuse([servo_upright, idler_upright])
    cuts = plate_cuts + [bearing_housing_cut(V(0, brg_out, 0), (0, -1, 0))]
    # pan horn on the underside (arm along Y)
    cuts.append(placed(horn_pocket(s["horn"], through=YOKE_FLOOR_T + 2), V(0, 0, floor_bot), (0, 1, 0), (0, 0, -1)))
    shape = shape.cut(Part.makeCompound(cuts)).removeSplitter()
    servo_env = placed(servo_envelope(s), t_origin, t_x, t_z)
    brg_env = bearing_envelope(V(0, brg_out, 0), (0, -1, 0))
    return shape, servo_env, brg_env, floor_bot


def base(yoke_floor_bot):
    s = STANDARD
    p_origin = V(0, 0, yoke_floor_bot - s["horn"]["hub"])
    p_x, p_z = (1, 0, 0), (0, 0, 1)
    plate, plate_cuts = servo_plate(s, (-48, 28), (-35, 35))
    plate = placed(plate, p_origin, p_x, p_z)
    plate_cuts = [placed(c, p_origin, p_x, p_z) for c in plate_cuts]
    env = placed(servo_envelope(s), p_origin, p_x, p_z)
    pb = plate.BoundBox
    foot_top = env.BoundBox.ZMin - 2.0
    foot = box(pb.XMin, pb.XMax, pb.YMin, pb.YMax, foot_top - 3, foot_top)
    wall_t = 3.0
    walls = [box(pb.XMin, pb.XMax, pb.YMin, pb.YMin + wall_t, foot_top - 3, pb.ZMax),
             box(pb.XMin, pb.XMax, pb.YMax - wall_t, pb.YMax, foot_top - 3, pb.ZMax),
             box(pb.XMin, pb.XMin + wall_t, pb.YMin, pb.YMax, foot_top - 3, pb.ZMax)]
    shape = plate.fuse([foot] + walls)
    cuts = plate_cuts
    # cable exit on the open +X end is free; bench screw holes in the foot corners
    for x in (pb.XMin + 8, pb.XMax - 8):
        for y in (pb.YMin + 8, pb.YMax - 8):
            cuts.append(Part.makeCylinder(1.75, 5, V(x, y, foot_top - 4)))
    return shape.cut(Part.makeCompound(cuts)).removeSplitter(), env


def carrier_assembly():
    """Carrier (and FeatherWing envelope) placed so the wing IC is at the origin."""
    shape = mc.carrier()
    ox, oy = mc.feather_origin()
    wing = Part.makeBox(mc.FEATHER_W, mc.FEATHER_H, 1.6, V(ox, oy, mc.THICKNESS + mc.BOSS_H))
    t = V(-PLATE_W / 2, -PLATE_H / 2, -CARRIER_IC_Z)
    shape.translate(t)
    wing.translate(t)
    return shape, wing


# --- clearance -------------------------------------------------------------
def rotated(shapes, axis, deg):
    out = []
    for s in shapes:
        c = s.copy()
        c.rotate(V(0, 0, 0), V(*axis), deg)
        out.append(c)
    return out


def sweep_check(moving, fixed, axis, label, step=15, tol=1e-3):
    moving_c = Part.makeCompound(moving)
    fixed_c = Part.makeCompound(fixed)
    worst = 0.0
    hits = []
    for deg in range(0, 360, step):
        m = moving_c.copy()
        m.rotate(V(0, 0, 0), V(*axis), deg)
        v = m.common(fixed_c).Volume
        if v > tol:
            hits.append((deg, round(v, 2)))
        worst = max(worst, v)
    print(f"clearance {label}: {'OK through 360 deg' if not hits else 'COLLIDES at ' + str(hits)}", flush=True)
    return hits


def build(variants=(16.0, 21.0), check=True):
    os.makedirs(PARTS_DIR, exist_ok=True)
    carrier, wing = carrier_assembly()
    cradle = roll_cradle()
    yoke, tilt_servo, yoke_brg, yoke_floor_bot = pan_yoke()
    base_shape, pan_servo = base(yoke_floor_bot)

    rings = {}
    for tab in variants:
        rings[tab] = tilt_ring(mg90s(tab))

    for name, shp in [("roll_cradle", cradle), ("pan_yoke", yoke), ("base", base_shape)] + \
            [(f"tilt_ring_mg90s_tab{int(t)}", r[0]) for t, r in rings.items()]:
        print(f"part {name}: valid {shp.isValid()} solids {len(shp.Solids)} bbox "
              f"{[round(v, 1) for v in (shp.BoundBox.XLength, shp.BoundBox.YLength, shp.BoundBox.ZLength)]}",
              flush=True)
        assert shp.isValid() and len(shp.Solids) == 1, name

    if check:
        roll_parts = [carrier, wing, cradle]
        for tab, (ring, roll_servo, ring_brg) in rings.items():
            print(f"-- MG90S tab {tab} mm", flush=True)
            sweep_check(roll_parts, [ring, roll_servo, ring_brg], (1, 0, 0), "roll: cradle+carrier vs ring")
            tilt_parts = roll_parts + [ring, roll_servo, ring_brg]
            sweep_check(tilt_parts, [yoke, tilt_servo, yoke_brg], (0, 1, 0), "tilt: ring assembly vs yoke")
        pan_parts = [yoke, tilt_servo, yoke_brg] + roll_parts + list(rings[variants[0]])
        sweep_check(pan_parts, [base_shape, pan_servo], (0, 0, 1), "pan: yoke assembly vs base")

    doc = App.newDocument("imu_gimbal_rig")
    def add(name, shp, label):
        o = doc.addObject("Part::Feature", name)
        o.Shape = shp
        o.Label = label
        o.Visibility = True
        return o

    add("Carrier", carrier, "Sensor carrier (swirly + Feather bosses)")
    add("FeatherWing", wing, "REF FeatherWing #4569 envelope")
    add("RollCradle", cradle, "Roll cradle")
    t0 = variants[0]
    add("TiltRing", rings[t0][0], f"Tilt ring (MG90S tab {t0} mm)")
    add("RollServo", rings[t0][1], f"REF MG90S roll servo (tab {t0} mm)")
    add("RollBearing", rings[t0][2], "REF 623ZZ roll idler")
    for t in variants[1:]:
        o = add(f"TiltRingTab{int(t)}", rings[t][0], f"Tilt ring (MG90S tab {t} mm) ALT")
        o.Visibility = False
    add("PanYoke", yoke, "Pan yoke")
    add("TiltServo", tilt_servo, "REF standard servo, tilt (MG995 / DS3240)")
    add("TiltBearing", yoke_brg, "REF 623ZZ tilt idler")
    add("Base", base_shape, "Base")
    add("PanServo", pan_servo, "REF standard servo, pan (DS3240 270)")
    doc.recompute()
    path = os.path.join(HERE, "imu-gimbal-assembly.FCStd")
    doc.saveAs(path)
    print("wrote", path, flush=True)

    for name, shp in [("roll_cradle", cradle), ("pan_yoke", yoke), ("base", base_shape)] + \
            [(f"tilt_ring_mg90s_tab{int(t)}", r[0]) for t, r in rings.items()]:
        p = os.path.join(PARTS_DIR, name)
        shp.exportStep(p + ".step")
        mesh = Mesh.Mesh(shp.tessellate(0.05))
        mesh.write(p + ".stl")
    print("wrote parts to", PARTS_DIR, flush=True)


if __name__ == "__main__":
    build()
