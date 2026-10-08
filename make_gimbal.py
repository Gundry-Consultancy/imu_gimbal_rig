"""Pan / tilt / roll gimbal around the sensor stack. Run with freecadcmd.

    freecadcmd -c "exec(open('make_gimbal.py').read(), {'__file__': 'make_gimbal.py', '__name__': '__main__'})"

Global frame: origin is the axis intersection (middle of the sensor stack), +X is the roll axis,
+Y is the tilt axis, +Z is the pan axis (up).

Parts (exported to parts/):
  stack_middle_deck, stack_top_deck, stack_bottom_deck, stack_spine_upper, stack_spine_lower
                                the sensor stack (see make_sensor_stack.py); only the middle deck's
                                end walls carry the MG90S horn (+X) and the 623 idler axle (-X)
  striker_bar, striker_pawl, striker_cam, striker_stand
                                cam-driven tap striker on the yoke (see make_striker.py)
  tilt_ring_mg90s                holds the roll MG90S and the -X 623 bearing;
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
import make_sensor_stack as ss  # noqa: E402

V = App.Vector
PARTS_DIR = os.path.join(HERE, "parts")

# --- servos ---------------------------------------------------------------
# Measured by hand 2026-10-08 (see docs/servo-measurements.png). Letters match the drawing.
# Readings run 0.1-0.2 mm generous (fit allowance), so they are used as-is.
# spline_above_tab (G) sets the mount position: the directly measured G is used
# (D - E - F reads ~0.8 mm more because the generous readings compound).
# offset: shaft centre from body centre along the long axis = A/2 - H.
MG90S_TAB = 16.8                                   # E, base to tab underside
MG90S_G = 12.0                                     # G, tab top to spline top (11.7-12 measured)
MG90S = dict(L=23.0,                               # A 22.7 measured, allow 23
             W=12.3, H=32.0, case_top=4.0,         # C, D
             tab_len=32.2, tab_t=2.5,              # B, F
             holes=(27.3, 27.7), hole_across=0.0,  # J 27.5 (+/-0.2 slot), single row
             pilot=1.7, offset=22.7 / 2 - 5.9,     # M 2.0 (M2 self-tap pilot), H 5.9
             spline_r=2.4,                         # N 21T, ~4.8
             horn=dict(len=36.0, width=7.0, depth=2.0, hub=2.5, centre=6.0))   # P-S not yet measured
DS3240 = dict(L=40.5, W=20.5, H=46.2, case_top=4.5,   # A, C, D
              tab_len=54.5, tab_t=3.2,                # B, F (3.08 measured)
              holes=(48.4, 48.8), hole_across=9.75,   # J 48.5-48.6, K 9.5-10
              pilot=2.4, offset=40.5 / 2 - 9.5,       # H ~9.5
              spline_r=3.0,
              spline_above_tab=14.0,                  # G measured (D - E - F reads 14.8)
              horn=dict(len=46.0, width=8.5, depth=2.5, hub=3.5, centre=7.0))  # P-S not yet measured
STANDARD = DS3240      # tilt and pan servos (an MG995 would need its own measurements)


def mg90s(tab_height):
    s = dict(MG90S)
    s["spline_above_tab"] = MG90S_G
    s["tab_height"] = tab_height
    return s


# --- bearing 623ZZ (3 x 10 x 4) -------------------------------------------
BRG_OD, BRG_W = 10.0, 4.0
BRG_POCKET = 10.15
LIP_HOLE = 6.5
AXLE_BOSS_D = 4.8   # touches inner race only
M3_PILOT = 2.8

# --- layout ------------------------------------------------------------------
BODY_HALF_X = ss.body_half_x()             # outer face of each cradle end wall
SWEEP_MARGIN = 4.0                          # beyond the measured sweep radius (cable slop)
RING_BAR_T = 6.0
RING_BAR_HALF_Z = 7.0
RING_BRG_PLATE_T = 6.0   # 4 mm bearing pocket + 2 mm lip
RING_GAP = 1.5

YOKE_PLATE_T = 6.0
YOKE_HALF_X = 16.0
YOKE_GAP = 1.5
YOKE_FLOOR_T = 6.0

SERVO_PLATE_T = 5.0
STAND_BOLTS = ((-11.0, 6.0), (11.0, 6.0), (-11.0, -12.0), (11.0, -12.0))   # striker stand on the idler upright (x, z)


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
def max_radius(shapes, axis):
    """Largest distance of any (tessellated) point from an axis through the origin."""
    a = V(*axis).normalize()
    r = 0.0
    for shp in shapes:
        for p in shp.tessellate(0.2)[0]:
            r = max(r, (p - a * p.dot(a)).Length)
    return r


def sensor_body():
    """Stack parts and board envelopes, moved so the roll axis is the X axis."""
    za = ss.axis_z()
    up = V(0, 0, za)
    horn = placed(horn_pocket(MG90S["horn"], through=ss.WALL_T + 2), V(BODY_HALF_X, 0, 0), (0, 1, 0), (1, 0, 0))
    boss_len = RING_GAP + (RING_BRG_PLATE_T - BRG_W)
    boss, pilot = axle_boss(V(-BODY_HALF_X, 0, 0), (-1, 0, 0), boss_len, boss_len + ss.WALL_T - 0.5)
    for c in (horn, boss, pilot):
        c.translate(up)
    upper, lower = ss.spines()
    parts = [("stack_middle_deck", ss.middle_deck(horn, boss, pilot)),
             ("stack_top_deck", ss.top_deck()),
             ("stack_bottom_deck", ss.bottom_deck()),
             ("stack_spine_upper", upper),
             ("stack_spine_lower", lower)]
    refs = ss.boards()
    for _, shp in parts + refs:
        shp.translate(-up)
    return parts, refs


def tilt_ring(servo, ring_in_y):
    ring_out_y = ring_in_y + RING_BAR_T
    roll_spline_x = BODY_HALF_X + servo["horn"]["hub"]
    s_origin, s_x, s_z = V(roll_spline_x, 0, 0), (0, 0, 1), (-1, 0, 0)
    # servo plate in servo-local coords: local x = world z, local y = world y, local z = -world x
    tab_z = servo["holes"][1] / 2 + 3
    plate, plate_cuts = servo_plate(servo, (-servo["offset"] - tab_z, -servo["offset"] + tab_z),
                                    (-ring_out_y, ring_out_y))
    plate = placed(plate, s_origin, s_x, s_z)
    plate_cuts = [placed(c, s_origin, s_x, s_z) for c in plate_cuts]
    plate_x_out = plate.BoundBox.XMax

    brg_in = -(BODY_HALF_X + RING_GAP)
    brg_out = brg_in - RING_BRG_PLATE_T
    brg_plate = box(brg_out, brg_in, -ring_out_y, ring_out_y, -RING_BAR_HALF_Z - 5, RING_BAR_HALF_Z + 5)

    bars = [box(brg_out, plate_x_out, y0, y1, -RING_BAR_HALF_Z, RING_BAR_HALF_Z)
            for y0, y1 in ((-ring_out_y, -ring_in_y), (ring_in_y, ring_out_y))]
    shape = plate.fuse([brg_plate] + bars)

    cuts = plate_cuts + [bearing_housing_cut(V(brg_out, 0, 0), (-1, 0, 0))]
    # +Y: tilt servo horn (arm along X)
    cuts.append(placed(horn_pocket(STANDARD["horn"], through=RING_BAR_T + 2), V(0, ring_out_y, 0), (1, 0, 0), (0, 1, 0)))
    # -Y: axle boss into the yoke bearing
    boss_len = YOKE_GAP + (YOKE_PLATE_T - BRG_W)
    boss, pilot = axle_boss(V(0, -ring_out_y, 0), (0, -1, 0), boss_len, boss_len + RING_BAR_T - 0.5)
    shape = shape.fuse(boss).cut(Part.makeCompound(cuts + [pilot]))
    servo_env = placed(servo_envelope(servo), s_origin, s_x, s_z)
    brg_env = bearing_envelope(V(brg_out, 0, 0), (-1, 0, 0))
    return shape.removeSplitter(), servo_env, brg_env


def pan_yoke(ring_out_y, tilt_clear_r):
    s = STANDARD
    floor_top = -tilt_clear_r
    floor_bot = floor_top - YOKE_FLOOR_T
    # tilt servo on +Y
    t_origin = V(0, ring_out_y + s["horn"]["hub"], 0)
    t_x, t_z = (0, 0, 1), (0, -1, 0)
    tab_half = s["tab_len"] / 2 + 3
    plate, plate_cuts = servo_plate(s, (-s["offset"] - tab_half, -s["offset"] + tab_half),
                                    (-YOKE_HALF_X, YOKE_HALF_X), thickness=YOKE_PLATE_T)
    plate = placed(plate, t_origin, t_x, t_z)
    # extend the servo plate down to the floor
    pb = plate.BoundBox
    servo_upright = box(-YOKE_HALF_X, YOKE_HALF_X, pb.YMin, pb.YMax, floor_bot, pb.ZMax)
    plate_cuts = [placed(c, t_origin, t_x, t_z) for c in plate_cuts]

    brg_in = -(ring_out_y + YOKE_GAP)
    brg_out = brg_in - YOKE_PLATE_T
    idler_upright = box(-YOKE_HALF_X, YOKE_HALF_X, brg_out, brg_in, floor_bot, 12)
    idler_upright = idler_upright.fuse(cyl(YOKE_HALF_X, V(0, brg_out, 12 - YOKE_HALF_X), V(0, brg_in, 12 - YOKE_HALF_X)))
    gusset_len = 25.0
    floor = box(-YOKE_HALF_X, YOKE_HALF_X, brg_out - gusset_len, pb.YMax + gusset_len, floor_bot, floor_top)
    gussets = []
    for x in (-YOKE_HALF_X, YOKE_HALF_X - 5):
        for y_face, out in ((brg_out, -1), (pb.YMax, 1)):
            pts = [V(x, y_face, floor_top), V(x, y_face + out * gusset_len, floor_top),
                   V(x, y_face, floor_top + gusset_len), V(x, y_face, floor_top)]
            gussets.append(Part.Face(Part.makePolygon(pts)).extrude(V(5, 0, 0)))

    shape = floor.fuse([servo_upright, idler_upright] + gussets)
    cuts = plate_cuts + [bearing_housing_cut(V(0, brg_out, 0), (0, -1, 0))]
    # M3 pilots for the striker stand on the idler upright's outer face
    cuts += [Part.makeCylinder(M3_PILOT / 2, 8, V(x, brg_out - 1, z), V(0, 1, 0)) for x, z in STAND_BOLTS]
    # pan horn on the underside (arm along Y)
    cuts.append(placed(horn_pocket(s["horn"], through=YOKE_FLOOR_T + 2), V(0, 0, floor_bot), (0, 1, 0), (0, 0, -1)))
    shape = shape.cut(Part.makeCompound(cuts)).removeSplitter()
    servo_env = placed(servo_envelope(s), t_origin, t_x, t_z)
    brg_env = bearing_envelope(V(0, brg_out, 0), (0, -1, 0))
    return shape, servo_env, brg_env, floor_bot, brg_out


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


def _self_module():
    import types
    m = types.ModuleType("make_gimbal_self")
    m.__dict__.update(globals())
    return m


def striker_limits(roll_parts, rings, striker_parked, striker_inst, yoke_parts, tol=1e-3):
    """Report gimbal range with the striker fitted (hammer parked) and check the installed hammer."""
    parked = Part.makeCompound(striker_parked)
    roll_c = Part.makeCompound(roll_parts)
    hits = []
    for deg in range(-90, 91, 10):
        m = roll_c.copy()
        m.rotate(V(0, 0, 0), V(1, 0, 0), deg)
        if m.common(parked).Volume > tol:
            hits.append(deg)
    print(f"striker vs roll +/-90 (hammer parked): {'OK' if not hits else 'COLLIDES at ' + str(hits)}", flush=True)
    for tab, r in rings.items():
        tilt_c = Part.makeCompound(roll_parts + list(r))
        limits = []
        for sign in (1, -1):
            ok = 0
            for deg in range(5, 181, 5):
                m = tilt_c.copy()
                m.rotate(V(0, 0, 0), V(0, 1, 0), sign * deg)
                if m.common(parked).Volume > tol:
                    break
                ok = deg
            limits.append(sign * ok)
        print(f"striker fitted, MG90S tab {tab}: tilt clear from {limits[1]} to +{limits[0]} deg", flush=True)
    rest = Part.makeCompound(striker_inst)
    v = rest.common(roll_c).Volume
    gap = rest.distToShape(roll_c)[0]
    print(f"hammer at rest: overlap {v:.3f} mm^3, gap to stack {gap:.2f} mm", flush=True)
    v = parked.common(Part.makeCompound(yoke_parts)).Volume
    print(f"striker vs yoke overlap {v:.3f} mm^3", flush=True)


def build(variants=(MG90S_TAB,), check=True):
    os.makedirs(PARTS_DIR, exist_ok=True)
    stack_parts, board_refs = sensor_body()
    roll_parts = [shp for _, shp in stack_parts] + [b for _, b in board_refs]
    roll_r = max_radius(roll_parts, (1, 0, 0))
    ring_in_y = roll_r + SWEEP_MARGIN
    print(f"roll sweep radius {roll_r:.1f} -> ring inner {ring_in_y:.1f}", flush=True)

    rings = {tab: tilt_ring(mg90s(tab), ring_in_y) for tab in variants}
    tilt_r = max(max_radius(roll_parts + list(r), (0, 1, 0)) for r in rings.values())
    tilt_clear_r = tilt_r + 3.0
    print(f"tilt sweep radius {tilt_r:.1f} -> yoke floor at z={-tilt_clear_r:.1f}", flush=True)

    yoke, tilt_servo, yoke_brg, yoke_floor_bot, upright_out_y = pan_yoke(ring_in_y + RING_BAR_T, tilt_clear_r)
    base_shape, pan_servo = base(yoke_floor_bot)

    import make_striker as mk
    st = mk.build(dict(upright_out_y=upright_out_y, roof_z=ss.roof_top() - ss.axis_z(),
                       upright_half_x=YOKE_HALF_X, stand_bolts=STAND_BOLTS), _self_module())
    print("striker", st["info"], flush=True)

    printed = stack_parts + [("pan_yoke", yoke), ("base", base_shape)] + \
              [("striker_bar", st["bar_free"]), ("striker_pawl", st["pawl_rest"]),
               ("striker_cam", st["cam"]), ("striker_stand", st["stand"])] + \
              [("tilt_ring_mg90s" if len(rings) == 1 else f"tilt_ring_mg90s_tab{t:g}", r[0]) for t, r in rings.items()]
    for name, shp in printed:
        print(f"part {name}: valid {shp.isValid()} solids {len(shp.Solids)} bbox "
              f"{[round(v, 1) for v in (shp.BoundBox.XLength, shp.BoundBox.YLength, shp.BoundBox.ZLength)]}",
              flush=True)
        assert shp.isValid() and len(shp.Solids) == 1, name

    if check:
        for tab, (ring, roll_servo, ring_brg) in rings.items():
            print(f"-- MG90S tab {tab} mm", flush=True)
            sweep_check(roll_parts, [ring, roll_servo, ring_brg], (1, 0, 0), "roll: sensor stack vs ring")
            sweep_check(roll_parts + [ring, roll_servo, ring_brg], [yoke, tilt_servo, yoke_brg], (0, 1, 0),
                        "tilt: ring assembly vs yoke")
        striker_parked = [st["bar_park"], st["pawl_park"], st["cam"], st["stand"], st["servo"]]
        pan_parts = [yoke, tilt_servo, yoke_brg] + roll_parts + list(rings[variants[0]]) + striker_parked
        sweep_check(pan_parts, [base_shape, pan_servo], (0, 0, 1), "pan: yoke + striker vs base")
        striker_limits(roll_parts, rings, striker_parked, [st["bar_rest"], st["pawl_rest"]],
                       [yoke, tilt_servo, yoke_brg])

    doc = App.newDocument("imu_gimbal_rig")

    def add(name, shp, label, visible=True):
        o = doc.addObject("Part::Feature", name)
        o.Shape = shp
        o.Label = label
        o.Visibility = visible
        return o

    labels = {"stack_middle_deck": "Middle deck + roll walls (wing + 5 QT)", "stack_top_deck": "Top deck (8 QT) + roof",
              "stack_bottom_deck": "Bottom deck (8 QT)", "stack_spine_upper": "Upper spine",
              "stack_spine_lower": "Lower spine"}
    for name, shp in stack_parts:
        add(name.title().replace("_", ""), shp, labels[name])
    for name, shp in board_refs:
        add(f"REF_{name}", shp, f"REF {name}")
    t0 = variants[0]
    add("TiltRing", rings[t0][0], f"Tilt ring (MG90S tab {t0} mm)")
    add("RollServo", rings[t0][1], f"REF MG90S roll servo (tab {t0} mm)")
    add("RollBearing", rings[t0][2], "REF 623ZZ roll idler")
    for t in variants[1:]:
        add(f"TiltRingTab{int(t)}", rings[t][0], f"Tilt ring (MG90S tab {t} mm) ALT", visible=False)
    add("PanYoke", yoke, "Pan yoke")
    add("TiltServo", tilt_servo, "REF standard servo, tilt (MG995 / DS3240)")
    add("TiltBearing", yoke_brg, "REF 623ZZ tilt idler")
    add("Base", base_shape, "Base")
    add("PanServo", pan_servo, "REF standard servo, pan (DS3240 270)")
    add("StrikerStand", st["stand"], "Striker stand")
    add("StrikerBar", st["bar_rest"], "Striker bar (at rest, hammer hovering)")
    add("StrikerPawl", st["pawl_rest"], "Striker pawl")
    add("StrikerCam", st["cam"], "Striker cam")
    add("StrikerServo", st["servo"], "REF MG90S-size 270 deg servo (cam)")
    add("StrikerBarParked", st["bar_park"], "REF striker bar parked (lifted)", visible=False)
    add("StrikerBarFree", st["bar_free"], "REF striker bar as printed (free shape)", visible=False)
    doc.recompute()
    path = os.path.join(HERE, "imu-gimbal-assembly.FCStd")
    doc.saveAs(path)
    print("wrote", path, flush=True)

    for name, shp in printed:
        p = os.path.join(PARTS_DIR, name)
        shp.exportStep(p + ".step")
        Mesh.Mesh(shp.tessellate(0.05)).write(p + ".stl")
    print("wrote parts to", PARTS_DIR, flush=True)


if __name__ == "__main__":
    build()
