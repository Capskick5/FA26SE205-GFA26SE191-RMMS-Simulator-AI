"""Bản đồ tầng dạng đồ thị + Dijkstra — làm phần việc backend sẽ làm (tính khoảng cách, tính đường)."""
import heapq
import json
import math
from pathlib import Path

MAP_DIR = Path(__file__).resolve().parent.parent / "simulator" / "map"


class FloorGraph:
    def __init__(self, floor="F2"):
        self.data = json.loads((MAP_DIR / f"{floor}.json").read_text(encoding="utf-8"))
        self.xy = {n["id"]: (n["x"], n["y"]) for n in self.data["nodes"]}
        self.adj = {n: [] for n in self.xy}
        for a, b in self.data["edges"]:
            d = math.dist(self.xy[a], self.xy[b])
            self.adj[a].append((b, d))
            self.adj[b].append((a, d))

    def dijkstra(self, start):
        best, prev, heap = {start: 0.0}, {}, [(0.0, start)]
        while heap:
            c, n = heapq.heappop(heap)
            if c > best[n]:
                continue
            for m, d in self.adj[n]:
                if c + d < best.get(m, math.inf):
                    best[m], prev[m] = c + d, n
                    heapq.heappush(heap, (c + d, m))
        return best, prev

    def distances(self, nodes):
        """Bảng khoảng cách (mét) giữa mọi cặp điểm trong `nodes` — đúng thứ AI cần."""
        nodes = sorted(set(nodes))
        out = {}
        for a in nodes:
            best, _ = self.dijkstra(a)
            out[a] = {b: round(best[b], 2) for b in nodes if b != a and b in best}
        return out

    def route(self, a, b):
        _, prev = self.dijkstra(a)
        path = [b]
        while path[-1] != a:
            path.append(prev[path[-1]])
        return path[::-1]

    def nearest(self, x, y):
        return min(self.xy, key=lambda n: math.dist((x, y), self.xy[n]))

    def path_for(self, start, stops):
        """Đường đi đầy đủ qua các điểm dừng, theo định dạng `path` trong contracts/mqtt.md."""
        path, cur = [], start
        for s in stops:
            nodes = self.route(cur, s["nodeId"])
            for k, n in enumerate(nodes):
                if path and path[-1]["nodeId"] == n and k < len(nodes) - 1:
                    continue
                item = {"nodeId": n, "x": self.xy[n][0], "y": self.xy[n][1]}
                if k == len(nodes) - 1:
                    item["stopSeq"] = s["seq"]
                path.append(item)
            cur = s["nodeId"]
        return path
