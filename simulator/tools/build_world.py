"""Sinh world Webots của một tầng từ map/<TẦNG>.json.

    python tools/build_world.py          # tầng F2 → worlds/rmms_F2.wbt
    python tools/build_world.py F3

KHÔNG sửa file .wbt bằng tay: đổi bản đồ thì sửa JSON (hoặc FE rồi chạy fe_to_map.py) và chạy lại lệnh này.
"""
import json
import math
import os
import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
FLOOR = sys.argv[1] if len(sys.argv) > 1 else "F2"
MAP = os.path.join(ROOT, "map", f"{FLOOR}.json")
WORLD = os.path.join(ROOT, "worlds", f"rmms_{FLOOR}.wbt")
TEXTURES = os.path.join(ROOT, "worlds", "textures")

WALL_H, WALL_T = 1.2, 0.15
WHEEL_R, TRACK = 0.08, 0.42
BODY, BODY_Z = (0.5, 0.36, 0.14), 0.08
PORT_GAP = 1.8                       # khoảng cách giữa các cổng của một trạm sạc (m)

KIND_COLOR = {"PHARMACY": (0.80, 0.89, 1.00), "STORAGE": (0.82, 0.95, 0.84), "SPECIMEN": (0.92, 0.86, 1.00),
              "DEPT": (0.95, 0.95, 0.93), "WASTE_STATION": (0.90, 0.85, 0.78)}
TYPE_COLOR = {"MED_CART": (0.15, 0.45, 0.90), "SUPPLY_CART": (0.15, 0.65, 0.35),
              "WASTE_CART": (0.55, 0.42, 0.28), "CLEAN_BOT": (0.10, 0.65, 0.70)}


# ---------------------------------------------------------------- helpers
def f(v):
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def vec(*vals):
    return " ".join(f(v) for v in vals)


def appearance(rgb, rough=0.7, metal=0.0, emissive=None, texture=None):
    s = f"PBRAppearance {{ baseColor {vec(*rgb)} roughness {f(rough)} metalness {f(metal)}"
    if emissive:
        s += f" emissiveColor {vec(*emissive)}"
    if texture:
        s += f' baseColorMap ImageTexture {{ url [ "{texture}" ] }}'
    return s + " }"


def box(size, rgb, **kw):
    return f"Shape {{ appearance {appearance(rgb, **kw)} geometry Box {{ size {vec(*size)} }} }}"


def cyl(h, r, rgb, **kw):
    return f"Shape {{ appearance {appearance(rgb, **kw)} geometry Cylinder {{ height {f(h)} radius {f(r)} }} }}"


def sphere(r, rgb, **kw):
    return f"Shape {{ appearance {appearance(rgb, **kw)} geometry Sphere {{ radius {f(r)} subdivision 2 }} }}"


def pose(t, child, rot=None):
    r = f" rotation {vec(*rot)}" if rot else ""
    return f"Pose {{ translation {vec(*t)}{r} children [ {child} ] }}"


def def_name(code):
    return code.replace("-", "_").replace(":", "_")


def axis_angle_view(yaw, pitch):
    """Hướng camera: quay quanh z (yaw) rồi cúi xuống (pitch)."""
    def q(axis, a):
        s = math.sin(a / 2)
        return (math.cos(a / 2), axis[0] * s, axis[1] * s, axis[2] * s)
    w1, x1, y1, z1 = q((0, 0, 1), yaw)
    w2, x2, y2, z2 = q((0, 1, 0), pitch)
    w, x, y, z = (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                  w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)
    ang = 2 * math.acos(max(-1.0, min(1.0, w)))
    s = math.sqrt(max(1e-12, 1 - w * w))
    return x / s, y / s, z / s, ang


def make_sign(code, name, color):
    """Biển tên phòng in lên sàn (cần Pillow; không có thì bỏ qua biển)."""
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        return None
    os.makedirs(TEXTURES, exist_ok=True)
    w, h = 1024, 256
    img = Image.new("RGB", (w, h), tuple(int(c * 255) for c in color))
    draw = ImageDraw.Draw(img)
    font_big = font_small = None
    for p in ("C:/Windows/Fonts/arialbd.ttf", "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"):
        if os.path.exists(p):
            font_big, font_small = ImageFont.truetype(p, 92), ImageFont.truetype(p, 60)
            break
    if font_big is None:
        font_big = font_small = ImageFont.load_default()
    draw.rectangle([6, 6, w - 7, h - 7], outline=(60, 60, 60), width=8)
    label = name.upper()
    if hasattr(font_big, "path"):                    # tên dài thì thu nhỏ chữ cho vừa biển
        size = 92
        while size > 40 and draw.textlength(label, font=font_big) > w - 60:
            size -= 4
            font_big = ImageFont.truetype(font_big.path, size)
    draw.text((w / 2, 100), label, fill=(25, 25, 25), font=font_big, anchor="mm")
    draw.text((w / 2, 200), code, fill=(90, 90, 90), font=font_small, anchor="mm")
    img.save(os.path.join(TEXTURES, f"sign_{code}.png"))
    return f"textures/sign_{code}.png"


# ---------------------------------------------------------------- robot
def distance_sensor(name, x, y, yaw):
    return (f'DistanceSensor {{ translation {vec(x, y, BODY_Z)} rotation 0 0 1 {f(yaw)} name "{name}" '
            f'lookupTable [ 0 0 0, 2 2 0 ] type "generic" }}')


def wheel(side, y):
    return f"""HingeJoint {{
      jointParameters HingeJointParameters {{ axis 0 1 0 anchor 0 {f(y)} 0 }}
      device [ RotationalMotor {{ name "{side}_wheel_motor" maxVelocity 20 maxTorque 20 }}
               PositionSensor {{ name "{side}_wheel_sensor" }} ]
      endPoint Solid {{
        translation 0 {f(y)} 0
        rotation 1 0 0 1.5708
        children [ {cyl(0.04, WHEEL_R, (0.08, 0.08, 0.08), rough=0.9)} ]
        name "{side}_wheel"
        boundingObject Cylinder {{ height 0.04 radius {f(WHEEL_R)} }}
        physics Physics {{ density -1 mass 0.8 }}
      }}
    }}"""


def caster(name, x):
    return f"""Solid {{
      translation {f(x)} 0 -0.04
      children [ {sphere(0.04, (0.3, 0.3, 0.3), rough=0.3, metal=0.5)} ]
      name "{name}"
      contactMaterial "caster"
      boundingObject Sphere {{ radius 0.04 }}
      physics Physics {{ density -1 mass 0.1 }}
    }}"""


def robot_top(rtype, color, slots):
    top = BODY_Z + BODY[2] / 2
    if rtype == "MED_CART":          # tủ có ngăn khoá, mỗi ngăn một vạch màu
        parts = [pose((0, 0, top + 0.15), box((0.40, 0.32, 0.30), (0.96, 0.96, 0.97)))]
        for i in range(max(1, slots)):
            z = top + 0.04 + i * (0.26 / max(1, slots))
            parts.append(pose((0.201, 0, z + 0.03), box((0.01, 0.28, 0.05), color)))
        return parts
    if rtype == "SUPPLY_CART":       # kệ nhiều tầng
        parts = [pose((-0.2, 0, top + 0.18), box((0.03, 0.34, 0.36), (0.6, 0.6, 0.6), metal=0.5))]
        for i in range(max(1, slots)):
            parts.append(pose((0, 0, top + 0.02 + i * 0.12), box((0.44, 0.34, 0.02), (0.3, 0.3, 0.3))))
        parts.append(pose((0.02, 0.03, top + 0.08), box((0.2, 0.15, 0.1), (0.85, 0.7, 0.45))))
        return parts
    if rtype == "WASTE_CART":        # thùng rác có nắp
        return [pose((0, 0, top + 0.16), cyl(0.32, 0.16, (0.95, 0.75, 0.10))),
                pose((0, 0, top + 0.33), cyl(0.03, 0.17, (0.25, 0.25, 0.25)))]
    # CLEAN_BOT: thân thấp + chổi lau phía trước
    return [pose((0.27, 0, 0.0), box((0.05, 0.40, 0.04), (0.2, 0.75, 0.85))),
            pose((-0.05, 0, top + 0.05), cyl(0.10, 0.14, (0.85, 0.95, 0.97)))]


def robot_node(robot, rtype_cfg, x, y, yaw):
    rtype = robot["type"]
    color = TYPE_COLOR.get(rtype, (0.5, 0.5, 0.5))
    top = BODY_Z + BODY[2] / 2
    children = [pose((0, 0, BODY_Z), box(BODY, color, rough=0.5)),
                pose((BODY[0] / 2 - 0.005, 0, BODY_Z + 0.03), box((0.02, 0.2, 0.03), (1, 1, 0.8), emissive=(0.9, 0.9, 0.6)))]
    children += robot_top(rtype, color, rtype_cfg.get("slots", 0))
    children += [
        distance_sensor("ds_front", BODY[0] / 2 + 0.01, 0, 0),
        distance_sensor("ds_front_left", BODY[0] / 2 + 0.01, 0.08, 0.26),
        distance_sensor("ds_front_right", BODY[0] / 2 + 0.01, -0.08, -0.26),
        distance_sensor("ds_diag_left", BODY[0] / 2, BODY[1] / 2 - 0.01, 0.70),
        distance_sensor("ds_diag_right", BODY[0] / 2, -BODY[1] / 2 + 0.01, -0.70),
        distance_sensor("ds_left", 0.15, BODY[1] / 2 + 0.01, 1.5708),
        distance_sensor("ds_right", 0.15, -BODY[1] / 2 - 0.01, -1.5708),
        f"""LED {{
      translation -0.2 0.12 {f(top + 0.04)}
      children [ Shape {{ appearance PBRAppearance {{ baseColor 0.9 0.9 0.9 roughness 0.3 metalness 0 emissiveColor 0 0 0 }}
                 geometry Sphere {{ radius 0.04 subdivision 2 }} }} ]
      name "status_led"
      color [ 0 1 0, 0 0.45 1, 1 0.8 0, 1 0 0, 1 1 1 ]
    }}""",
        'GPS { name "gps" }',
        'Compass { name "compass" }',
        wheel("left", TRACK / 2), wheel("right", -TRACK / 2),
        caster("front_caster", 0.19), caster("rear_caster", -0.19),
    ]
    body = "\n    ".join(children)
    return f"""DEF {def_name(robot["id"])} Robot {{
  translation {vec(x, y, WHEEL_R + 0.002)}
  rotation 0 0 1 {f(yaw)}
  children [
    {body}
  ]
  name "{robot["id"]}"
  model "{rtype}"
  controller "rmms_robot"
  controllerArgs [ "--floor={FLOOR}" "--home={robot["home"]}" "--speed={rtype_cfg.get('speed', 0.7)}" ]
  boundingObject Pose {{ translation 0 0 {f(BODY_Z)} children [ Box {{ size {vec(*BODY)} }} ] }}
  physics Physics {{ density -1 mass 15 centerOfMass [ 0 0 0.02 ] }}
}}"""


# ---------------------------------------------------------------- tường
def wall_segments(rooms, W, H):
    segs = set()

    def add(x0, y0, x1, y1):
        if (abs(x0 - x1) < 1e-6 and abs(x0) < 1e-6) or (abs(x0 - x1) < 1e-6 and abs(x0 - W) < 1e-6):
            return
        if (abs(y0 - y1) < 1e-6 and abs(y0) < 1e-6) or (abs(y0 - y1) < 1e-6 and abs(y0 - H) < 1e-6):
            return
        a, b = sorted([(round(x0, 3), round(y0, 3)), (round(x1, 3), round(y1, 3))])
        if a != b:
            segs.add((a, b))

    for r in rooms:
        x0, y0, x1, y1 = r["rect"]
        door = r["door"]
        sides = {"S": (x0, y0, x1, y0), "N": (x0, y1, x1, y1), "W": (x0, y0, x0, y1), "E": (x1, y0, x1, y1)}
        for side, (ax, ay, bx, by) in sides.items():
            if side == door["side"]:
                half = door["width"] / 2
                add(ax, ay, door["x"] - half, by)
                add(door["x"] + half, ay, bx, by)
            else:
                add(ax, ay, bx, by)
    return sorted(segs)


def solid_box(name, cx, cy, sx, sy, h, rgb):
    return f"""Solid {{
  translation {vec(cx, cy, h / 2)}
  children [ {box((sx, sy, h), rgb)} ]
  name "{name}"
  boundingObject Box {{ size {vec(sx, sy, h)} }}
  locked TRUE
}}"""


# ---------------------------------------------------------------- build
def build():
    data = json.load(open(MAP, encoding="utf-8"))
    W, H = data["size"]
    locs = {l["code"]: l for l in data["locations"]}
    rooms = [l for l in data["locations"] if "rect" in l]
    out = ["#VRML_SIM R2023b utf8", ""]
    out.append(f"""WorldInfo {{
  info [
    "RMMS — tầng {FLOOR}. Sinh tự động từ map/{FLOOR}.json bằng tools/build_world.py. KHÔNG sửa tay."
    "Toạ độ: x = đông, y = bắc, mét; gốc = góc trái-dưới khung tầng của FE (10 px FE = 1 m)."
  ]
  title "RMMS {FLOOR}"
  basicTimeStep 32
  contactProperties [ ContactProperties {{ material2 "caster" coulombFriction [ 0 ] }} ]
}}""")
    # camera nhìn từ phía nam, chếch xuống: thấy trọn tầng
    cx, cy = W / 2, H / 2
    pitch, dist = math.radians(58), max(W, H * 2.2) * 1.32
    out.append(f"Viewpoint {{\n  orientation {vec(*axis_angle_view(math.pi / 2, pitch))}\n"
               f"  position {vec(cx, cy - dist * math.cos(pitch), dist * math.sin(pitch))}\n  near 0.5\n}}")
    out.append("Background { skyColor [ 0.86 0.90 0.93 ] }")
    out.append("DirectionalLight { ambientIntensity 0.7 direction -0.3 0.4 -1 intensity 1.1 castShadows FALSE }")

    out.append(f"""DEF FLOOR Solid {{
  translation {vec(cx, cy, -0.05)}
  children [ {box((W, H, 0.1), (0.70, 0.72, 0.73), rough=0.9)} ]
  name "floor"
  boundingObject Box {{ size {vec(W, H, 0.1)} }}
  locked TRUE
}}""")
    for c in data["corridors"]:
        x0, y0, x1, y1 = c["rect"]
        out.append(pose(((x0 + x1) / 2, (y0 + y1) / 2, 0.006), box((x1 - x0, y1 - y0, 0.008), (0.92, 0.93, 0.93), rough=1)))
    for r in rooms:
        x0, y0, x1, y1 = r["rect"]
        color = KIND_COLOR.get(r["kind"], (0.93, 0.93, 0.93))
        out.append(pose(((x0 + x1) / 2, (y0 + y1) / 2, 0.006), box((x1 - x0, y1 - y0, 0.008), color, rough=1)))
        tex = make_sign(r["code"], r["name"], color)
        if tex:
            sy = y1 - 2.0 if r["door"]["side"] == "S" else y0 + 2.0       # biển nằm phía trong, xa cửa
            sw = x1 - x0 - 2.0
            out.append(pose(((x0 + x1) / 2, sy, 0.025),
                            f"Shape {{ appearance {appearance((1, 1, 1), rough=1, texture=tex)} "
                            f"geometry Plane {{ size {vec(sw, sw / 4)} }} }}"))

    for i, ((ax, ay), (bx, by)) in enumerate(wall_segments(rooms, W, H)):
        length = math.hypot(bx - ax, by - ay)
        sx, sy = (length + WALL_T, WALL_T) if abs(ay - by) < 1e-6 else (WALL_T, length + WALL_T)
        out.append(solid_box(f"wall_{i}", (ax + bx) / 2, (ay + by) / 2, sx, sy, WALL_H, (0.93, 0.93, 0.90)))
    for name, x, y, sx, sy in [("outer_s", cx, 0, W + WALL_T, WALL_T), ("outer_n", cx, H, W + WALL_T, WALL_T),
                               ("outer_w", 0, cy, WALL_T, H), ("outer_e", W, cy, WALL_T, H)]:
        out.append(solid_box(name, x, y, sx, sy, WALL_H, (0.80, 0.80, 0.78)))

    # trạm sạc: mỗi cổng một tấm đỗ + tấm lưng (robot đỗ quay mặt ra hành lang)
    homes = {}
    corr_y = (data["corridors"][0]["rect"][1] + data["corridors"][0]["rect"][3]) / 2
    for l in data["locations"]:
        if l["kind"] not in ("CHARGER", "CLEAN_STATION"):
            continue
        ports = l.get("ports", 1)
        in_corridor = abs(l["y"] - corr_y) < 1
        for p in range(ports):
            off = (p - (ports - 1) / 2) * PORT_GAP
            if in_corridor:                          # trạm ở cuối hành lang: đỗ quay mặt về phía tây
                px, py, yaw, back = l["x"], l["y"] + off, math.pi, (0.85, 0)
            else:
                faces_south = l["y"] > corr_y        # phòng phía bắc → quay mặt về nam (ra cửa)
                px, py = l["x"] + off, l["y"]
                yaw = -math.pi / 2 if faces_south else math.pi / 2
                back = (0, 0.85 if faces_south else -0.85)
            homes.setdefault(l["code"], []).append((px, py, yaw))
            pad_color = (0.3, 0.85, 0.9) if l["kind"] == "CLEAN_STATION" else (1, 0.85, 0.1)
            out.append(pose((px, py, 0.022), box((0.8, 0.8, 0.008), pad_color, rough=1)))
            bsx, bsy = (0.15, 0.9) if in_corridor else (0.9, 0.15)
            out.append(f"""Solid {{
  translation {vec(px + back[0], py + back[1], 0.25)}
  children [ {box((bsx, bsy, 0.5), (0.2, 0.2, 0.22), metal=0.4)} ]
  name "charger_{def_name(l["code"])}_{p + 1}"
  boundingObject Box {{ size {vec(bsx, bsy, 0.5)} }}
  locked TRUE
}}""")

    # điểm gom rác: dấu trên sàn (không có vật cản, robot đi qua được)
    for l in data["locations"]:
        if l["kind"] == "WASTE_POINT":
            out.append(pose((l["x"], l["y"], 0.03), cyl(0.01, 0.35, (0.75, 0.55, 0.2), rough=1), rot=(1, 0, 0, 0)))

    # robot ở trạm nhà
    used = {}
    for robot in data["robots"]:
        slots = homes.get(robot["home"])
        if not slots:
            h = locs[robot["home"]]
            slots = homes[robot["home"]] = [(h["x"], h["y"], 0.0)]
        i = used.get(robot["home"], 0)
        used[robot["home"]] = i + 1
        x, y, yaw = slots[i % len(slots)]
        if i >= len(slots):                          # nhiều robot hơn số cổng: đứng chờ cạnh trạm
            x += 1.2 * (i - len(slots) + 1)
        out.append(robot_node(robot, data["robot_types"].get(robot["type"], {}), x, y, yaw))

    os.makedirs(os.path.dirname(WORLD), exist_ok=True)
    with open(WORLD, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(out) + "\n")
    print(f"{WORLD}: {len(rooms)} phòng, {len(wall_segments(rooms, W, H))} đoạn tường, {len(data['robots'])} robot")


if __name__ == "__main__":
    build()
