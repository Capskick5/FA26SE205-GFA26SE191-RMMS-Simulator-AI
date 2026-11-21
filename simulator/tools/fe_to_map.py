"""Lấy bản đồ một tầng từ FE (src/legacy/engine.js) → map/<TẦNG>.json, đơn vị mét.

    python tools/fe_to_map.py              # tầng F2
    python tools/fe_to_map.py F3

Quy đổi giống FE: 10 px = 1 m. FE vẽ y hướng xuống, Webots dùng y hướng lên (bắc) nên y được lật:
    x_m = x_px / 10        y_m = (440 - y_px) / 10
Đồ thị đường đi (node + edge) dựng ĐÚNG luật graphOf() của FE, để Webots, backend và FE đi cùng một đồ thị.
"""
import json
import math
import os
import re
import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
FE_ENGINE = os.environ.get("FE_ENGINE", r"D:\FPT\CODE_Capstone\Frontend\src\legacy\engine.js")
FLOOR = sys.argv[1] if len(sys.argv) > 1 else "F2"

PX_PER_M = 10.0
H_PX, W_PX = 440, 1020          # khung tầng của FE: phòng từ y 30→410, x 40→980
CORR_Y_PX = 220                 # trục hành lang của FE
CORR_HALF_PX = 30               # hành lang từ y 190 → 250 (rộng 6 m)
DOOR_W = 2.0                    # cửa phòng (m)


def m(x_px, y_px):
    return round(x_px / PX_PER_M, 2), round((H_PX - y_px) / PX_PER_M, 2)


def parse_fe(src):
    text = open(src, encoding="utf-8").read()
    rooms, points, robots = [], [], []
    for code, name, floor, kind, row, slot in re.findall(
            r"room\('([^']+)','([^']+)','([^']+)','([^']+)','([TB])',(\d+)\)", text):
        if floor != FLOOR:
            continue
        x = 40 + int(slot) * 160
        top = row == "T"
        # giống hàm room() của FE: khung 140×160 px, điểm dừng (x+70, 182 | 258) sát cửa phía hành lang
        rooms.append({"code": code, "name": name, "kind": kind, "rx": x, "ry": 30 if top else 250, "rw": 140, "rh": 160,
                      "x": x + 70, "y": 182 if top else 258, "side": "S" if top else "N"})
    for code, name, floor, kind, x, y in re.findall(
            r"pt\('([^']+)','([^']+)','([^']+)','([^']+)',(\d+),(\d+)\)", text):
        if floor == FLOOR:
            points.append({"code": code, "name": name, "kind": kind, "x": int(x), "y": int(y)})
    ports = dict(re.findall(r"'([A-Z0-9-]+)':(\d+)", re.search(r"const PORTS=\{([^}]*)\}", text).group(1)))
    for rid, rtype, floor, home in re.findall(r"addRobot\('([^']+)','([^']+)','([^']+)','([^']+)'", text):
        if floor == FLOOR:
            robots.append({"id": rid, "type": rtype, "home": home})
    rtypes = {}
    for name, body in re.findall(r"(\w+):\{name:'[^']*'(.*?)\}", re.search(r"const RTYPE = \{(.*?)\n\};", text, re.S).group(1)):
        def num(key, default=0):
            mm = re.search(key + r":([\d.]+)", body)
            return float(mm.group(1)) if mm else default
        label = re.search(rf"{name}:\{{name:'([^']*)'", text).group(1)
        rtypes[name] = {"name": label, "slots": int(num("slots")), "slot_kg": num("slotKg"),
                        "speed": num("speed", 0.7), "payload": num("payload")}
    return rooms, points, {k: int(v) for k, v in ports.items()}, robots, rtypes


def build_graph(rooms, points):
    """Chép luật graphOf() trong FE engine.js."""
    locs = rooms + points
    n, edges, chain = {}, [], []

    def add(nid, x, y, real):
        n[nid] = {"id": nid, "x": x, "y": y, "real": real}

    def link(a, b):
        edges.append([a, b])

    for l in locs:
        add(l["code"], l["x"], l["y"], True)
    for l in locs:
        if abs(l["y"] - CORR_Y_PX) <= 1:
            chain.append(l["code"])
            continue
        if "rw" in l:
            j = "J:" + l["code"]
            add(j, l["x"], CORR_Y_PX, False)
            link(l["code"], j)
            chain.append(j)
            continue
        same_side = [r for r in rooms if abs(r["x"] - l["x"]) < 75 and ((r["y"] < CORR_Y_PX) == (l["y"] < CORR_Y_PX))]
        host = sorted(same_side, key=lambda r: abs(r["x"] - l["x"]))[0] if same_side else None
        if host:
            link(l["code"], host["code"])
            continue
        j = "J:" + l["code"]
        add(j, l["x"], CORR_Y_PX, False)
        link(l["code"], j)
        chain.append(j)
    chain.sort(key=lambda c: n[c]["x"])
    for a, b in zip(chain, chain[1:]):
        link(a, b)
    return n, edges


def main():
    rooms, points, ports, robots, rtypes = parse_fe(FE_ENGINE)
    if not rooms:
        sys.exit(f"Không thấy phòng nào của tầng {FLOOR} trong {FE_ENGINE}")
    nodes, edges = build_graph(rooms, points)

    out_locs = []
    for r in rooms:
        x0, y1 = m(r["rx"], r["ry"])
        x1, y0 = m(r["rx"] + r["rw"], r["ry"] + r["rh"])
        px, py = m(r["x"], r["y"])
        door_y = y0 if r["side"] == "S" else y1
        out_locs.append({"code": r["code"], "name": r["name"], "kind": r["kind"], "x": px, "y": py,
                         "rect": [x0, y0, x1, y1], "door": {"side": r["side"], "x": px, "y": door_y, "width": DOOR_W}})
    for p in points:
        x, y = m(p["x"], p["y"])
        entry = {"code": p["code"], "name": p["name"], "kind": p["kind"], "x": x, "y": y}
        if p["kind"] in ("CHARGER", "CLEAN_STATION"):
            entry["ports"] = ports.get(p["code"], 1)
        out_locs.append(entry)

    cx0, cy0 = m(0, CORR_Y_PX + CORR_HALF_PX)
    cx1, cy1 = m(W_PX, CORR_Y_PX - CORR_HALF_PX)
    data = {
        "floor": FLOOR,
        "source": "Sinh từ FE src/legacy/engine.js (prototype v8.2) bằng tools/fe_to_map.py — đừng sửa tay, sửa FE rồi chạy lại.",
        "units": "m",
        "coordinates": "x hướng đông, y hướng bắc; gốc (0,0) = góc trái-dưới khung tầng FE; 10 px FE = 1 m",
        "size": [W_PX / PX_PER_M, H_PX / PX_PER_M],
        "corridors": [{"code": f"{FLOOR}-CORRIDOR", "name": "Hành lang chính", "rect": [cx0, cy0, cx1, cy1]}],
        "locations": out_locs,
        "nodes": [{"id": v["id"], "x": m(v["x"], v["y"])[0], "y": m(v["x"], v["y"])[1], "real": v["real"]}
                  for v in nodes.values()],
        "edges": edges,
        "robot_types": rtypes,
        "robots": robots,
    }
    os.makedirs(os.path.join(ROOT, "map"), exist_ok=True)
    path = os.path.join(ROOT, "map", f"{FLOOR}.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, ensure_ascii=False, indent=1)
    total = sum(math.dist((nodes[a]["x"], nodes[a]["y"]), (nodes[b]["x"], nodes[b]["y"])) for a, b in edges) / PX_PER_M
    print(f"{path}: {len(rooms)} phòng, {len(points)} điểm, {len(nodes)} node, {len(edges)} đoạn ({total:.0f} m đường), "
          f"{len(robots)} robot: {', '.join(r['id'] for r in robots)}")


if __name__ == "__main__":
    main()
