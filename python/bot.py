"""Balanced offense + active defense (v9 4.22 core + reload cover-dip & strafe-boost) for mechfight/1."""
from __future__ import annotations

import array
import heapq
import json
import math
import os
import random
import sys
import time
from typing import Any, Dict, List, Optional, Set, Tuple

import mechfight as mf


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def angle_diff(a: float, b: float) -> float:
    return abs(wrap_angle(a - b))


def horiz_dist(a: List[float], b: List[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def dist3(a: List[float], b: List[float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


# ---------------------------------------------------------------------------
# Map geometry, terrain, exact wall proximity, 3D LOS, and A*
# ---------------------------------------------------------------------------

class Prism:
    __slots__ = ("kind", "base", "top", "ring", "holes", "min_x", "max_x", "min_y", "max_y", "edges")

    def __init__(self, raw: Dict[str, Any]) -> None:
        self.kind: str = raw["kind"]
        self.base: float = float(raw["base"])
        self.top: float = float(raw["top"])
        self.ring: List[Tuple[float, float]] = self._to_pts(raw["ring"])
        self.holes: List[List[Tuple[float, float]]] = [
            h_pts for h in raw.get("holes", []) if len(h_pts := self._to_pts(h)) >= 3
        ]
        xs = [p[0] for p in self.ring]
        ys = [p[1] for p in self.ring]
        self.min_x = min(xs) if xs else 0.0
        self.max_x = max(xs) if xs else 0.0
        self.min_y = min(ys) if ys else 0.0
        self.max_y = max(ys) if ys else 0.0
        self.edges: List[Tuple[float, float, float, float]] = []
        self._add_edges(self.ring)
        for h in self.holes:
            self._add_edges(h)

    @staticmethod
    def _to_pts(flat: List[float]) -> List[Tuple[float, float]]:
        return [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat) - 1, 2)]

    def _add_edges(self, pts: List[Tuple[float, float]]) -> None:
        n = len(pts)
        for i in range(n):
            x0, y0 = pts[i]
            x1, y1 = pts[(i + 1) % n]
            if x0 != x1 or y0 != y1:
                self.edges.append((x0, y0, x1, y1))


def _point_in_ring(x: float, y: float, pts: List[Tuple[float, float]]) -> bool:
    n = len(pts)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def _point_in_prism_2d(x: float, y: float, p: Prism) -> bool:
    if x < p.min_x or x > p.max_x or y < p.min_y or y > p.max_y:
        return False
    if not _point_in_ring(x, y, p.ring):
        return False
    for h in p.holes:
        if _point_in_ring(x, y, h):
            return False
    return True


def _segs_intersect(
    ax: float, ay: float, bx: float, by: float,
    cx: float, cy: float, dx: float, dy: float,
) -> bool:
    r_x, r_y = bx - ax, by - ay
    s_x, s_y = dx - cx, dy - cy
    denom = r_x * s_y - r_y * s_x
    if abs(denom) < 1e-9:
        return False
    qpx, qpy = cx - ax, cy - ay
    t = (qpx * s_y - qpy * s_x) / denom
    if t < 0.0 or t > 1.0:
        return False
    u = (qpx * r_y - qpy * r_x) / denom
    return 0.0 <= u <= 1.0


def _dist_point_to_seg_sq(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    c1 = vx * wx + vy * wy
    if c1 <= 0.0:
        return wx * wx + wy * wy
    c2 = vx * vx + vy * vy
    if c2 <= c1:
        dx, dy = px - bx, py - by
        return dx * dx + dy * dy
    b = c1 / c2
    qx, qy = ax + b * vx, ay + b * vy
    dx, dy = px - qx, py - qy
    return dx * dx + dy * dy


class WorldMap:
    def __init__(self, map_dir: str, mech_cfg: Dict[str, float]) -> None:
        self.loaded = False
        self.bounds = {"minX": -512.0, "maxX": 555.0, "minY": -333.0, "maxY": 388.0}
        self.prisms: List[Prism] = []
        self.terrain: Optional[array.array] = None
        self.t_min_x = -513.0
        self.t_min_y = -334.0
        self.t_step = 1.0
        self.t_nx = 0
        self.t_ny = 0
        self.t_scale = 0.01

        self.step_up = float(mech_cfg.get("step_up", 1.2))
        self.mech_h = float(mech_cfg.get("height", 9.0))
        self.max_slope_tan = math.tan(math.radians(float(mech_cfg.get("max_slope_deg", 50.0)) - 3.0))

        self.bucket_size = 20.0
        self.buckets: Dict[Tuple[int, int], List[int]] = {}

        self.nav_step = 5.0
        self.nav_nx = 0
        self.nav_ny = 0
        self.nav_min_x = 0.0
        self.nav_min_y = 0.0
        self.nav_blocked = bytearray()
        self.nav_tall_wall = bytearray()
        self.nav_penalty: List[float] = []

        world_path = os.path.join(map_dir, "v2", "world.json")
        terrain_path = os.path.join(map_dir, "v2", "terrain.i16")
        if not os.path.exists(world_path):
            return

        with open(world_path, "r", encoding="utf-8") as f:
            w = json.load(f)

        self.bounds = w["bounds"]
        t_info = w["terrain"]
        self.t_min_x = float(t_info["minX"])
        self.t_min_y = float(t_info["minY"])
        self.t_step = float(t_info["step"])
        self.t_nx = int(t_info["nx"])
        self.t_ny = int(t_info["ny"])
        self.t_scale = float(t_info["scale"])

        if os.path.exists(terrain_path):
            with open(terrain_path, "rb") as tf:
                arr = array.array("h")
                arr.frombytes(tf.read())
                if sys.byteorder != "little":
                    arr.byteswap()
                self.terrain = arr

        for rp in w.get("prisms", []):
            if len(rp.get("ring", [])) >= 6:
                p = Prism(rp)
                idx = len(self.prisms)
                self.prisms.append(p)
                bx0 = int(math.floor(p.min_x / self.bucket_size))
                bx1 = int(math.floor(p.max_x / self.bucket_size))
                by0 = int(math.floor(p.min_y / self.bucket_size))
                by1 = int(math.floor(p.max_y / self.bucket_size))
                for bx in range(bx0, bx1 + 1):
                    for by in range(by0, by1 + 1):
                        self.buckets.setdefault((bx, by), []).append(idx)

        self._build_nav_grid()
        self.loaded = True

    def terrain_z(self, x: float, y: float) -> float:
        if self.terrain is None or self.t_nx <= 1 or self.t_ny <= 1:
            return 0.0
        fx = (x - self.t_min_x) / self.t_step
        fy = (y - self.t_min_y) / self.t_step
        i = int(clamp(math.floor(fx), 0, self.t_nx - 2))
        j = int(clamp(math.floor(fy), 0, self.t_ny - 2))
        tx = clamp(fx - i, 0.0, 1.0)
        ty = clamp(fy - j, 0.0, 1.0)
        nx = self.t_nx
        t = self.terrain
        z00 = t[j * nx + i]
        z10 = t[j * nx + i + 1]
        z01 = t[(j + 1) * nx + i]
        z11 = t[(j + 1) * nx + i + 1]
        z0 = z00 + (z10 - z00) * tx
        z1 = z01 + (z11 - z01) * tx
        return (z0 + (z1 - z0) * ty) * self.t_scale

    def _build_nav_grid(self) -> None:
        self.nav_min_x = float(self.bounds["minX"])
        self.nav_min_y = float(self.bounds["minY"])
        self.nav_nx = int((self.bounds["maxX"] - self.nav_min_x) / self.nav_step) + 1
        self.nav_ny = int((self.bounds["maxY"] - self.nav_min_y) / self.nav_step) + 1
        total = self.nav_nx * self.nav_ny
        blocked = bytearray(total)
        tall_wall = bytearray(total)

        nx, ny = self.nav_nx, self.nav_ny
        min_x, min_y, s = self.nav_min_x, self.nav_min_y, self.nav_step
        b_max_x = float(self.bounds["maxX"]) - 3.0
        b_max_y = float(self.bounds["maxY"]) - 3.0
        b_min_x = float(self.bounds["minX"]) + 3.0
        b_min_y = float(self.bounds["minY"]) + 3.0

        for gy in range(ny):
            y = min_y + (gy + 0.5) * s
            row = gy * nx
            for gx in range(nx):
                x = min_x + (gx + 0.5) * s
                if x < b_min_x or x > b_max_x or y < b_min_y or y > b_max_y:
                    blocked[row + gx] = 1
                    continue
                if self.terrain is not None:
                    zc = self.terrain_z(x, y)
                    dzx = abs(self.terrain_z(x + 2.0, y) - zc) * 0.5
                    dzy = abs(self.terrain_z(x, y + 2.0) - zc) * 0.5
                    if max(dzx, dzy) > self.max_slope_tan:
                        blocked[row + gx] = 1

        clear_r = 2.7
        clear_r_sq = clear_r * clear_r
        for p in self.prisms:
            gx0 = max(0, int((p.min_x - clear_r - min_x) / s))
            gx1 = min(nx - 1, int((p.max_x + clear_r - min_x) / s))
            gy0 = max(0, int((p.min_y - clear_r - min_y) / s))
            gy1 = min(ny - 1, int((p.max_y + clear_r - min_y) / s))
            for gy in range(gy0, gy1 + 1):
                y = min_y + (gy + 0.5) * s
                row = gy * nx
                for gx in range(gx0, gx1 + 1):
                    x = min_x + (gx + 0.5) * s
                    tz = self.terrain_z(x, y)
                    if (p.base - tz) >= self.mech_h or (p.top - tz) <= self.step_up:
                        continue
                    is_tall = (p.top - tz) >= 8.5
                    if _point_in_prism_2d(x, y, p):
                        blocked[row + gx] = 1
                        if is_tall:
                            tall_wall[row + gx] = 1
                        continue
                    if not blocked[row + gx]:
                        for ax, ay, bx, by in p.edges:
                            if _dist_point_to_seg_sq(x, y, ax, ay, bx, by) < clear_r_sq:
                                blocked[row + gx] = 1
                                if is_tall:
                                    tall_wall[row + gx] = 1
                                break

        self.nav_blocked = blocked
        self.nav_tall_wall = tall_wall

        dist = [-1] * total
        q: List[int] = []
        for idx, b in enumerate(blocked):
            if b:
                dist[idx] = 0
                q.append(idx)
        head = 0
        while head < len(q):
            cur = q[head]
            head += 1
            d = dist[cur]
            if d >= 2:
                continue
            cx = cur % nx
            cy = cur // nx
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxp, nyp = cx + dx, cy + dy
                if 0 <= nxp < nx and 0 <= nyp < ny:
                    nidx = nyp * nx + nxp
                    if dist[nidx] == -1:
                        dist[nidx] = d + 1
                        q.append(nidx)

        penalty = [0.0] * total
        for idx, d in enumerate(dist):
            if d == 0:
                penalty[idx] = 25.0
            elif d == 1:
                penalty[idx] = 2.0
            elif d == 2:
                penalty[idx] = 0.4
        self.nav_penalty = penalty

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int(clamp((x - self.nav_min_x) / self.nav_step, 0, self.nav_nx - 1))
        gy = int(clamp((y - self.nav_min_y) / self.nav_step, 0, self.nav_ny - 1))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        return (
            self.nav_min_x + (gx + 0.5) * self.nav_step,
            self.nav_min_y + (gy + 0.5) * self.nav_step,
        )

    def is_blocked(self, x: float, y: float) -> bool:
        if not self.loaded:
            return False
        gx, gy = self.world_to_grid(x, y)
        return bool(self.nav_blocked[gy * self.nav_nx + gx])

    def point_near_wall(self, x: float, y: float, radius: float = 2.7) -> bool:
        if not self.loaded:
            return False
        if (
            x < float(self.bounds["minX"]) + radius
            or x > float(self.bounds["maxX"]) - radius
            or y < float(self.bounds["minY"]) + radius
            or y > float(self.bounds["maxY"]) - radius
        ):
            return True
        tz = self.terrain_z(x, y)
        r_sq = radius * radius
        bx = int(math.floor(x / self.bucket_size))
        by = int(math.floor(y / self.bucket_size))
        seen: Set[int] = set()
        for obx in range(bx - 1, bx + 2):
            for oby in range(by - 1, by + 2):
                for pidx in self.buckets.get((obx, oby), ()):
                    if pidx in seen:
                        continue
                    seen.add(pidx)
                    p = self.prisms[pidx]
                    if (p.base - tz) >= self.mech_h or (p.top - tz) <= self.step_up:
                        continue
                    if x < p.min_x - radius or x > p.max_x + radius or y < p.min_y - radius or y > p.max_y + radius:
                        continue
                    if _point_in_prism_2d(x, y, p):
                        return True
                    for ax, ay, cx, cy in p.edges:
                        if _dist_point_to_seg_sq(x, y, ax, ay, cx, cy) < r_sq:
                            return True
        return False

    def has_near_tall_wall(self, x: float, y: float) -> bool:
        if not self.loaded:
            return False
        gx, gy = self.world_to_grid(x, y)
        nx, ny = self.nav_nx, self.nav_ny
        for dy in (-2, -1, 0, 1, 2):
            yy = gy + dy
            if 0 <= yy < ny:
                row = yy * nx
                for dx in (-2, -1, 0, 1, 2):
                    xx = gx + dx
                    if 0 <= xx < nx and self.nav_tall_wall[row + xx]:
                        return True
        return False

    def _grid_line_clear(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        d = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(d / (self.nav_step * 0.5)))
        for k in range(1, steps + 1):
            t = k / steps
            if self.is_blocked(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t):
                return False
        return True

    def find_next_waypoint(self, sx: float, sy: float, gx_w: float, gy_w: float) -> Tuple[float, float]:
        if not self.loaded:
            return gx_w, gy_w
        sgx, sgy = self.world_to_grid(sx, sy)
        tgx, tgy = self.world_to_grid(gx_w, gy_w)
        nx, ny = self.nav_nx, self.nav_ny
        blocked = self.nav_blocked
        penalty = self.nav_penalty

        if blocked[tgy * nx + tgx]:
            best = None
            best_d2 = 1e9
            for r in range(1, 7):
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        cx, cy = tgx + dx, tgy + dy
                        if 0 <= cx < nx and 0 <= cy < ny and not blocked[cy * nx + cx]:
                            d2 = dx * dx + dy * dy
                            if d2 < best_d2:
                                best_d2 = d2
                                best = (cx, cy)
                if best is not None:
                    tgx, tgy = best
                    break

        if sgx == tgx and sgy == tgy:
            return gx_w, gy_w

        start_idx = sgy * nx + sgx
        goal_idx = tgy * nx + tgx

        if math.hypot(gx_w - sx, gy_w - sy) < 95.0 and not blocked[start_idx] and self._grid_line_clear(sx, sy, gx_w, gy_w):
            return gx_w, gy_w

        g_score: Dict[int, float] = {start_idx: 0.0}
        came_from: Dict[int, int] = {}
        pq: List[Tuple[float, int]] = [(0.0, start_idx)]
        visited: Set[int] = set()
        neighbors = (
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142),
        )

        best_node = start_idx
        best_h = 1e9
        pops = 0
        start_blocked = bool(blocked[start_idx])

        while pq and pops < 1000:
            _, cur = heapq.heappop(pq)
            if cur in visited:
                continue
            visited.add(cur)
            pops += 1
            if cur == goal_idx:
                best_node = cur
                break

            cx = cur % nx
            cy = cur // nx
            dx_g = abs(cx - tgx)
            dy_g = abs(cy - tgy)
            h_cur = dx_g + dy_g - 0.5858 * min(dx_g, dy_g)
            if h_cur < best_h:
                best_h = h_cur
                best_node = cur

            cur_g = g_score[cur]
            cur_is_blocked = bool(blocked[cur])
            for ox, oy, step_c in neighbors:
                nxp, nyp = cx + ox, cy + oy
                if not (0 <= nxp < nx and 0 <= nyp < ny):
                    continue
                nidx = nyp * nx + nxp
                if nidx in visited:
                    continue
                if blocked[nidx] and not (start_blocked and cur_is_blocked):
                    continue
                if ox != 0 and oy != 0:
                    if (blocked[cy * nx + nxp] or blocked[nyp * nx + cx]) and not cur_is_blocked:
                        continue
                ng = cur_g + step_c + penalty[nidx]
                if ng < g_score.get(nidx, 1e18):
                    g_score[nidx] = ng
                    came_from[nidx] = cur
                    dxh = abs(nxp - tgx)
                    dyh = abs(nyp - tgy)
                    h = 1.4 * (dxh + dyh - 0.5858 * min(dxh, dyh))
                    heapq.heappush(pq, (ng + h, nidx))

        if best_node == start_idx:
            return gx_w, gy_w

        path: List[Tuple[float, float]] = []
        cur = best_node
        while cur != start_idx:
            path.append(self.grid_to_world(cur % nx, cur // nx))
            cur = came_from[cur]
        path.reverse()
        if best_node == goal_idx:
            path.append((gx_w, gy_w))

        chosen = path[0]
        if not start_blocked:
            for wx, wy in path:
                if math.hypot(wx - sx, wy - sy) > 38.0:
                    break
                if self._grid_line_clear(sx, sy, wx, wy):
                    chosen = (wx, wy)
                else:
                    break
        elif len(path) >= 2:
            chosen = path[min(2, len(path) - 1)]
        return chosen

    def has_los(self, p1: List[float], p2: List[float]) -> bool:
        if not self.loaded:
            return True
        x1, y1, z1 = p1[0], p1[1], p1[2]
        x2, y2, z2 = p2[0], p2[1], p2[2]
        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1
        hdist = math.hypot(dx, dy)

        if self.terrain is not None and hdist > 4.0:
            steps = max(2, int(hdist / 5.0))
            for k in range(1, steps):
                t = k / steps
                if z1 + dz * t < self.terrain_z(x1 + dx * t, y1 + dy * t) - 0.25:
                    return False

        bx0 = int(math.floor(min(x1, x2) / self.bucket_size))
        bx1 = int(math.floor(max(x1, x2) / self.bucket_size))
        by0 = int(math.floor(min(y1, y2) / self.bucket_size))
        by1 = int(math.floor(max(y1, y2) / self.bucket_size))

        seen: Set[int] = set()
        r_min_x, r_max_x = min(x1, x2), max(x1, x2)
        r_min_y, r_max_y = min(y1, y2), max(y1, y2)
        r_min_z, r_max_z = min(z1, z2), max(z1, z2)

        for bx in range(bx0, bx1 + 1):
            for by in range(by0, by1 + 1):
                plist = self.buckets.get((bx, by))
                if not plist:
                    continue
                for pidx in plist:
                    if pidx in seen:
                        continue
                    seen.add(pidx)
                    p = self.prisms[pidx]
                    if p.max_x < r_min_x or p.min_x > r_max_x or p.max_y < r_min_y or p.min_y > r_max_y:
                        continue
                    p_base = p.base + 0.15
                    p_top = p.top - 0.15
                    if p_top < r_min_z or p_base > r_max_z or p_top <= p_base:
                        continue

                    if abs(dz) < 1e-6:
                        if z1 < p_base or z1 > p_top:
                            continue
                        t_lo, t_hi = 0.0, 1.0
                    else:
                        ta = (p_base - z1) / dz
                        tb = (p_top - z1) / dz
                        t_lo = max(0.0, min(ta, tb))
                        t_hi = min(1.0, max(ta, tb))
                        if t_lo > t_hi:
                            continue

                    sx1 = x1 + dx * t_lo
                    sy1 = y1 + dy * t_lo
                    sx2 = x1 + dx * t_hi
                    sy2 = y1 + dy * t_hi
                    if _point_in_prism_2d(sx1, sy1, p) or _point_in_prism_2d(sx2, sy2, p):
                        return False
                    for ax, ay, cx, cy in p.edges:
                        if _segs_intersect(sx1, sy1, sx2, sy2, ax, ay, cx, cy):
                            return False
        return True


# ---------------------------------------------------------------------------
# Ballistic & aiming helpers
# ---------------------------------------------------------------------------

def solve_lead_aim(
    muzzle: List[float], target_pos: List[float], target_vel: List[float],
    speed: float, max_time: float = 0.70,
) -> Tuple[float, float]:
    dx = target_pos[0] - muzzle[0]
    dy = target_pos[1] - muzzle[1]
    dz = target_pos[2] - muzzle[2]
    vx, vy, vz = target_vel[0], target_vel[1], target_vel[2]

    a = vx * vx + vy * vy + vz * vz - speed * speed
    b = 2.0 * (dx * vx + dy * vy + dz * vz)
    c = dx * dx + dy * dy + dz * dz

    t = math.sqrt(c) / max(speed, 1.0)
    if abs(a) < 1e-5:
        if abs(b) > 1e-5:
            t_lin = -c / b
            if t_lin > 0.0:
                t = t_lin
    else:
        disc = b * b - 4.0 * a * c
        if disc >= 0.0:
            sq = math.sqrt(disc)
            t1 = (-b - sq) / (2.0 * a)
            t2 = (-b + sq) / (2.0 * a)
            cands = [x for x in (t1, t2) if x > 0.0]
            if cands:
                t = min(cands)
    t = clamp(t, 0.0, max_time)
    px = dx + vx * t
    py = dy + vy * t
    pz = dz + vz * t
    return math.atan2(py, px), math.atan2(pz, max(1e-4, math.hypot(px, py)))


def solve_mortar_aim(
    muzzle: List[float], target_pos: List[float], target_vel: List[float],
    speed: float, gravity: float, world_map: WorldMap,
    shot_origin_dist: float = 3.0, lead_scale: float = 0.85,
    prefer_high: bool = False,
) -> Optional[Tuple[float, float]]:
    base_z = target_pos[2] + 2.2
    flight_t = horiz_dist(muzzle, target_pos) / max(speed * 0.82, 1.0)
    pred = [target_pos[0], target_pos[1], base_z]
    p_est = 0.25
    for _ in range(4):
        lt = clamp(flight_t, 0.0, 3.0) * lead_scale + 0.05
        pred = [
            target_pos[0] + target_vel[0] * lt,
            target_pos[1] + target_vel[1] * lt,
            base_z + target_vel[2] * lt * 0.45,
        ]
        dx, dy = pred[0] - muzzle[0], pred[1] - muzzle[1]
        r_raw = math.hypot(dx, dy)
        if r_raw < 16.0:
            return None
        r = max(8.0, r_raw - shot_origin_dist * math.cos(p_est))
        dz = pred[2] - (muzzle[2] + shot_origin_dist * math.sin(p_est))
        v2 = speed * speed
        disc = v2 * v2 - gravity * (gravity * r * r + 2.0 * dz * v2)
        if disc < 0.0:
            return None
        sq = math.sqrt(disc)
        p_est = math.atan2(v2 + sq, gravity * r) if prefer_high else math.atan2(v2 - sq, gravity * r)
        vx_h = speed * math.cos(p_est)
        if vx_h > 1.0:
            flight_t = r / vx_h

    dx, dy = pred[0] - muzzle[0], pred[1] - muzzle[1]
    r_raw = math.hypot(dx, dy)
    if r_raw < 16.0:
        return None
    yaw = math.atan2(dy, dx)

    cand_pitches: List[float] = []
    for use_hi in ((True, False) if prefer_high else (False, True)):
        p_cur = 0.9 if use_hi else 0.2
        valid = True
        for _ in range(2):
            r = max(8.0, r_raw - shot_origin_dist * math.cos(p_cur))
            dz = pred[2] - (muzzle[2] + shot_origin_dist * math.sin(p_cur))
            v2 = speed * speed
            disc = v2 * v2 - gravity * (gravity * r * r + 2.0 * dz * v2)
            if disc < 0.0:
                valid = False
                break
            sq = math.sqrt(disc)
            p_cur = math.atan2(v2 + sq, gravity * r) if use_hi else math.atan2(v2 - sq, gravity * r)
        if valid and (-1.2 <= p_cur <= 1.35):
            cand_pitches.append(p_cur)

    for pitch in cand_pitches:
        cos_p, sin_p = math.cos(pitch), math.sin(pitch)
        if cos_p < 0.05:
            continue
        r = max(8.0, r_raw - shot_origin_dist * cos_p)
        total_t = r / (speed * cos_p)
        vx = speed * cos_p * math.cos(yaw)
        vy = speed * cos_p * math.sin(yaw)
        vz = speed * sin_p
        ox = muzzle[0] + shot_origin_dist * cos_p * math.cos(yaw)
        oy = muzzle[1] + shot_origin_dist * cos_p * math.sin(yaw)
        oz = muzzle[2] + shot_origin_dist * sin_p
        if not world_map.has_los(muzzle, [ox, oy, oz]):
            continue
        ok = True
        prev = [ox, oy, oz]
        for k in range(1, 9):
            st = (k / 9.0) * total_t * 0.88
            nxt = [ox + vx * st, oy + vy * st, oz + vz * st - 0.5 * gravity * st * st]
            if not world_map.has_los(prev, nxt):
                ok = False
                break
            prev = nxt
        if ok:
            return yaw, pitch
    return None


# ---------------------------------------------------------------------------
# Bot controller
# ---------------------------------------------------------------------------

WEAPON_PRIORITY = {"laser": 4, "mortar": 3, "lightning": 2, "gun": 1}


class Bot:
    name = "hybrid-v15"

    def __init__(self, hello: mf.Hello) -> None:
        self.cfg = hello["config"]
        self.rng = random.Random(int(hello.get("seed", 42)))
        self.pickup_radius = float(self.cfg["loot"]["pickup_radius"])
        self.mech_cfg = self.cfg["mech"]
        self.walk_speed = float(self.mech_cfg.get("walk_speed", 11.7))
        self.min_pitch = float(self.mech_cfg.get("min_pitch", -1.2))
        self.max_pitch = float(self.mech_cfg.get("max_pitch", 1.2))
        self.size_per_cube = float(self.cfg.get("cubes", {}).get("size_per_cube", 0.05))
        self.cube_stat_cap = int(self.cfg.get("cubes", {}).get("stat_cap", 10))
        self.upgrade_thresholds = set(self.cfg.get("cubes", {}).get("upgrade_thresholds", [1, 3, 6, 10, 15, 21]))
        self.upgrades_def: Dict[str, Dict[str, Any]] = {
            u["id"]: u for u in self.cfg.get("upgrades", [])
        }

        map_dir = hello.get("map", {}).get("dir", "/arena")
        self.world = WorldMap(map_dir, self.mech_cfg)

        self.threats: Dict[int, Dict[str, Any]] = {}
        self.prev_vel: Dict[int, Tuple[float, List[float]]] = {}
        self.direct_hit_by_t: Dict[int, float] = {}
        self.recent_sounds: List[Tuple[float, float, str]] = []
        self.strafe_dir = 1 if self.rng.random() < 0.5 else -1
        self.last_flip_t = 0.0
        self.next_strafe_flip_t = 0.0

        self.cached_goal: Optional[Tuple[float, float]] = None
        self.cached_wp: Optional[Tuple[float, float]] = None
        self.last_path_t = -10.0
        self.cached_cover: Optional[Tuple[float, float]] = None
        self.last_cover_t = -10.0

        self.last_pos: Optional[List[float]] = None
        self.stuck_score = 0.0
        self.unstuck_until = -1.0

    def _pick_upgrade(self, obs: mf.Observation) -> int:
        offer = obs["offer"]
        if not offer:
            return 0
        me = obs["self"]
        hp_frac = float(me["hp"]) / max(1.0, float(me["max_hp"]))
        held_kinds = {w["kind"] for w in me["weapons"] if w is not None}
        best_i, best_score = 0, -1e9
        for idx, choice in enumerate(offer["choices"]):
            uid = choice["id"]
            udef = self.upgrades_def.get(uid)
            score = 10.0
            if udef:
                req = udef.get("requires")
                if req and req not in held_kinds:
                    score -= 60.0
                elif req == "lightning" and ("laser" not in held_kinds and "mortar" not in held_kinds):
                    score -= 25.0

                for eff in udef.get("effects", []):
                    path = eff.get("path", "")
                    mul = float(eff.get("mul", 1.0))
                    add = float(eff.get("add", 0.0))
                    # Balanced offense + defense: gun > armor > laser > max_hp > speed
                    if req == "gun" or path.startswith("gun"):
                        score += 132.0 if ("rate" in uid or "cooldown" in path or "damage" in path) else 96.0
                    elif "damage_mult" in path or "armor" in path:
                        score += 142.0 if hp_frac < 0.70 else 128.0
                    elif req == "laser" or path.startswith("laser"):
                        score += 118.0 if "charge" in path else 104.0
                    elif "max_hp" in path:
                        score += 115.0
                    elif "walk_speed" in path or "boost_speed" in path:
                        score += 105.0
                    elif req == "mortar" or path.startswith("mortar"):
                        score += 98.0
                    elif "regen_delay" in path or "regen_per_s" in path:
                        score += 92.0
                    elif "energy" in path or "jet" in path:
                        score += 86.0
                    elif req == "lightning" or path.startswith("lightning"):
                        score += 68.0
                    elif "near_range" in path or "bearing_noise" in path or "vision" in path:
                        score += 65.0
                    elif "turn_rate" in path or "pitch_rate" in path:
                        score += 55.0
                    else:
                        score += abs(mul - 1.0) * 40.0 + abs(add) * 2.0
            else:
                desc = (choice.get("name", "") + " " + choice.get("desc", "")).lower()
                if "gun" in desc or "fire rate" in desc:
                    score += 130.0
                elif "armor" in desc or "damage taken" in desc:
                    score += 126.0
                elif "hp" in desc or "speed" in desc:
                    score += 108.0
            if score > best_score:
                best_score, best_i = score, idx
        return best_i

    def _choose_loot_action(self, obs: mf.Observation) -> Dict[str, int]:
        me = obs["self"]
        mx, my, mz = me["pos"]
        weapons = me["weapons"]
        held_kinds = [w["kind"] if w else None for w in weapons]

        if any(w and w["kind"] == "laser" and w.get("charge", 0.0) > 0.0 for w in weapons):
            return {}

        nearby = [
            p for p in obs["pickups"]
            if math.hypot(p["pos"][0] - mx, p["pos"][1] - my) <= self.pickup_radius - 0.15
            and abs(p["pos"][2] - mz) <= 3.7
            and p["kind"] not in held_kinds
        ]
        if not nearby:
            return {}

        nearby.sort(key=lambda p: WEAPON_PRIORITY.get(p["kind"], 0), reverse=True)
        best_p = nearby[0]
        p_prio = WEAPON_PRIORITY.get(best_p["kind"], 0)

        for s in (1, 0):
            if weapons[s] is None:
                return {"take": int(best_p["id"])}

        worst_slot = 1
        if WEAPON_PRIORITY.get(weapons[0]["kind"], 0) < WEAPON_PRIORITY.get(weapons[1]["kind"], 0):
            worst_slot = 0
        if p_prio > WEAPON_PRIORITY.get(weapons[worst_slot]["kind"], 0):
            return {"take": int(best_p["id"]), "drop": worst_slot}
        return {}

    @staticmethod
    def _zone_circle(obs: mf.Observation) -> Tuple[List[float], float]:
        z = obs["zone"]
        t = obs["t"]
        s0, s1 = z["shrink_start"], z["shrink_end"]
        if t <= s0 or s1 <= s0:
            return list(z["center"]), float(z["radius"])
        if t >= s1:
            return list(z["next_center"]), float(z["next_radius"])
        frac = (t - s0) / (s1 - s0)
        return [
            z["center"][0] + (z["next_center"][0] - z["center"][0]) * frac,
            z["center"][1] + (z["next_center"][1] - z["center"][1]) * frac,
        ], float(z["radius"] + (z["next_radius"] - z["radius"]) * frac)

    def _update_awareness(self, obs: mf.Observation) -> Optional[float]:
        t = obs["t"]
        me = obs["self"]
        mx, my, mz = me["pos"]
        rear_bearing: Optional[float] = None

        for ev in obs["events"]:
            if ev["type"] == "damaged":
                if ev.get("rear") and ev.get("bearing") is not None:
                    rear_bearing = float(ev["bearing"])
                by = ev.get("by")
                if by is not None:
                    bid = int(by)
                    if ev.get("weapon") in ("gun", "laser", "lightning"):
                        self.direct_hit_by_t[bid] = t
                    if bid not in self.threats and ev.get("bearing") is not None:
                        b = float(ev["bearing"])
                        self.threats[bid] = {
                            "id": bid,
                            "pos": [mx + 45.0 * math.cos(b), my + 45.0 * math.sin(b), mz],
                            "vel": [0.0, 0.0, 0.0],
                            "accel": 0.0,
                            "aim": [b + math.pi, 0.0],
                            "hp": 1400.0,
                            "weapons": [],
                            "charging": False,
                            "cubes": 0.0,
                            "t": t,
                            "visible": False,
                        }

        visible_ids: Set[int] = set()
        for m in obs["mechs"]:
            if m["kind"] != "mech":
                continue
            mid = int(m["id"])
            visible_ids.add(mid)
            vel = list(m["vel"])
            accel_mag = 0.0
            if mid in self.prev_vel:
                pt, pv = self.prev_vel[mid]
                dt = t - pt
                if 0.04 <= dt <= 0.25:
                    accel_mag = math.hypot((vel[0] - pv[0]) / dt, (vel[1] - pv[1]) / dt)
            self.prev_vel[mid] = (t, vel)

            self.threats[mid] = {
                "id": mid,
                "pos": list(m["pos"]),
                "vel": vel,
                "accel": accel_mag,
                "aim": list(m["aim"]),
                "hp": float(m.get("hp", 1000.0)),
                "weapons": list(m.get("weapons", [])),
                "charging": bool(m.get("charging", False)),
                "cubes": float(m.get("cubes", 0.0)),
                "t": t,
                "visible": True,
            }

        for beam in obs["beams"]:
            by = int(beam["by"])
            if by not in visible_ids and beam.get("path"):
                origin = list(beam["path"][0])
                origin[2] = max(origin[2] - 7.0, self.world.terrain_z(origin[0], origin[1]))
                prev = self.threats.get(by, {})
                self.threats[by] = {
                    "id": by,
                    "pos": origin,
                    "vel": [0.0, 0.0, 0.0],
                    "accel": 0.0,
                    "aim": prev.get("aim", [0.0, 0.0]),
                    "hp": prev.get("hp", 1000.0),
                    "weapons": prev.get("weapons", [beam["kind"]]),
                    "charging": False,
                    "cubes": prev.get("cubes", 0.0),
                    "t": t - float(beam.get("age", 0.0)),
                    "visible": False,
                }

        stale = [k for k, v in self.threats.items() if t - v["t"] > 5.5]
        for k in stale:
            del self.threats[k]
        for k, v in self.threats.items():
            if k not in visible_ids:
                v["visible"] = False

        self.recent_sounds = [(st, sb, sd) for (st, sb, sd) in self.recent_sounds if t - st < 1.5]
        for s in obs["sounds"]:
            if s["kind"] in ("gun", "mortar", "explosion", "laser", "laser_charge", "lightning", "step", "boost"):
                if s["dist"] != "far":
                    self.recent_sounds.append((t - float(s["age"]), float(s["bearing"]), s["dist"]))

        return rear_bearing

    def _dodge_vector(self, obs: mf.Observation) -> Tuple[float, float, bool, bool]:
        me = obs["self"]
        mx, my, mz = me["pos"]
        scale = 1.0 + self.size_per_cube * me["cubes"]
        com_z = mz + 4.5 * scale
        dodge_x, dodge_y = 0.0, 0.0
        urgent_boost = False
        urgent_jump = False
        g = float(self.cfg["weapons"]["mortar"].get("gravity", 22.0))

        for p in obs["projectiles"][:16]:
            px, py, pz = p["pos"]
            vx, vy, vz = p["vel"]
            if p["kind"] == "mortar":
                for step_i in range(1, 24):
                    st = step_i * 0.08
                    sx, sy = px + vx * st, py + vy * st
                    sz = pz + vz * st - 0.5 * g * st * st
                    if sz <= mz + 1.0:
                        d_imp = math.hypot(mx - sx, my - sy)
                        if d_imp < 9.5:
                            away_x, away_y = mx - sx, my - sy
                            ln = math.hypot(away_x, away_y) or 1.0
                            w = (10.0 - d_imp) * 0.45
                            dodge_x += (away_x / ln) * w
                            dodge_y += (away_y / ln) * w
                            if st < 0.75 and d_imp < 7.5:
                                urgent_boost = True
                                if st < 0.35 and me["grounded"]:
                                    urgent_jump = True
                        break
            else:
                v2 = vx * vx + vy * vy + vz * vz
                if v2 < 1.0:
                    continue
                dx, dy, dz = mx - px, my - py, com_z - pz
                t_ca = (dx * vx + dy * vy + dz * vz) / v2
                if 0.0 < t_ca < 0.65:
                    cx, cy, cz = px + vx * t_ca, py + vy * t_ca, pz + vz * t_ca
                    d_ca = math.sqrt((mx - cx) ** 2 + (my - cy) ** 2 + (com_z - cz) ** 2)
                    if d_ca < 4.7 * scale:
                        hv = math.hypot(vx, vy) or 1.0
                        px_n, py_n = -vy / hv, vx / hv
                        sign = 1.0 if ((mx - cx) * px_n + (my - cy) * py_n) >= 0.0 else -1.0
                        w = (4.9 * scale - d_ca) * 0.6
                        dodge_x += px_n * sign * w
                        dodge_y += py_n * sign * w

        for th in self.threats.values():
            if not th["visible"] or not th["charging"]:
                continue
            tx, ty, _ = th["pos"]
            ayaw = th["aim"][0]
            ux, uy = math.cos(ayaw), math.sin(ayaw)
            rx, ry = mx - tx, my - ty
            proj = rx * ux + ry * uy
            if 0.0 < proj < 270.0:
                perp_x, perp_y = rx - proj * ux, ry - proj * uy
                d_perp = math.hypot(perp_x, perp_y)
                if d_perp < 6.8:
                    if d_perp > 0.1:
                        dodge_x += (perp_x / d_perp) * 1.5
                        dodge_y += (perp_y / d_perp) * 1.5
                    else:
                        dodge_x += -uy * 1.5
                        dodge_y += ux * 1.5
                    urgent_boost = True

        return dodge_x, dodge_y, urgent_boost, urgent_jump

    def _find_cover_pos(self, obs: mf.Observation) -> Tuple[float, float]:
        t = obs["t"]
        if self.cached_cover is not None and t - self.last_cover_t < 0.8:
            return self.cached_cover

        me = obs["self"]
        mx, my, _ = me["pos"]
        eye_h = float(me["stats"].get("eye_height", 8.0))
        nc = obs["zone"]["next_center"]
        nr = max(8.0, float(obs["zone"]["next_radius"]))

        active_threats = sorted(
            (th for th in self.threats.values() if t - th["t"] < 4.5),
            key=lambda th: math.hypot(th["pos"][0] - mx, th["pos"][1] - my),
        )[:2]

        best_pos = (nc[0], nc[1])
        best_score = -1e9
        for dist_r in (16.0, 32.0):
            for k in range(8):
                ang = k * (math.pi / 4.0)
                cx = mx + dist_r * math.cos(ang)
                cy = my + dist_r * math.sin(ang)
                if self.world.point_near_wall(cx, cy, 2.7):
                    continue
                cz = self.world.terrain_z(cx, cy)
                c_eye = [cx, cy, cz + eye_h]
                score = 0.0
                d_next = math.hypot(cx - nc[0], cy - nc[1])
                score += 65.0 if d_next <= max(8.0, nr * 0.9) else -(d_next - nr) * 1.8
                for th in active_threats:
                    tx, ty, tz = th["pos"]
                    t_scale = 1.0 + self.size_per_cube * th.get("cubes", 0.0)
                    if not self.world.has_los(c_eye, [tx, ty, tz + 8.0 * t_scale]):
                        score += 135.0
                    score += min(math.hypot(cx - tx, cy - ty), 95.0) * 0.45
                score -= dist_r * 0.4
                if score > best_score:
                    best_score = score
                    best_pos = (cx, cy)

        self.cached_cover = best_pos
        self.last_cover_t = t
        return best_pos

    def _safe_anchor(self, obs: mf.Observation) -> Tuple[float, float]:
        me = obs["self"]
        mx, my, _ = me["pos"]
        nc = obs["zone"]["next_center"]
        nr = max(8.0, float(obs["zone"]["next_radius"]))
        dx, dy = mx - nc[0], my - nc[1]
        d = math.hypot(dx, dy)
        max_r = max(4.0, nr * 0.78)
        if d <= max_r:
            return mx, my
        ang = math.atan2(dy, dx) if d > 1.0 else 0.0
        target_r = min(nr * 0.62, max(0.0, nr - 15.0))
        return nc[0] + target_r * math.cos(ang), nc[1] + target_r * math.sin(ang)

    def _has_mech_los(self, t: float, my_eye: List[float], m: Dict[str, Any]) -> bool:
        if t - self.direct_hit_by_t.get(int(m["id"]), -10.0) < 0.45:
            return True
        t_scale = 1.0 + self.size_per_cube * float(m.get("cubes", 0.0))
        mp = m["pos"]
        return (
            self.world.has_los(my_eye, [mp[0], mp[1], mp[2] + 4.5 * t_scale])
            or self.world.has_los(my_eye, [mp[0], mp[1], mp[2] + 8.6 * t_scale])
        )

    def _eval_loot_target(
        self, obs: mf.Observation, cur_zc: List[float], cur_zr: float,
        has_los_target: bool, d_target_2d: float,
    ) -> Optional[Tuple[float, float, str]]:
        me = obs["self"]
        mx, my, _ = me["pos"]
        hp_frac = me["hp"] / max(1.0, me["max_hp"])
        cubes = int(me["cubes"])
        nc = obs["zone"]["next_center"]
        nr = max(8.0, float(obs["zone"]["next_radius"]))
        held_kinds = {w["kind"] for w in me["weapons"] if w is not None}
        need_weapon = len(held_kinds) < 2
        worst_prio = min((WEAPON_PRIORITY.get(k, 0) for k in held_kinds), default=0)

        def risk_divisor(ix: float, iy: float, exposure_s: float) -> float:
            d_me = math.hypot(ix - mx, iy - my)
            if math.hypot(ix - cur_zc[0], iy - cur_zc[1]) > cur_zr - 12.0:
                return 1e9
            if obs["t"] >= obs["zone"]["shrink_start"] - 6.0 and math.hypot(ix - nc[0], iy - nc[1]) > nr * 0.92:
                return 1e9
            risk = 10.0 + d_me + exposure_s * 12.0
            for th in self.threats.values():
                d_th = math.hypot(ix - th["pos"][0], iy - th["pos"][1])
                if d_th < d_me * 0.95 or d_th < 45.0:
                    risk += 180.0
                elif d_th < 85.0:
                    risk += 55.0
            if hp_frac < 0.60:
                risk *= 1.5
            return risk

        best_goal: Optional[Tuple[float, float, str]] = None
        best_roi = 0.0

        hits_upgrade = (cubes + 1) in self.upgrade_thresholds
        cube_val = 155.0 if cubes < self.cube_stat_cap else (115.0 if hits_upgrade else 18.0)
        if hits_upgrade and cubes < self.cube_stat_cap:
            cube_val += 50.0
        for c in obs["cubes"]:
            cx, cy = c["pos"][0], c["pos"][1]
            d = math.hypot(cx - mx, cy - my)
            if has_los_target and d > 15.0:
                continue
            roi = cube_val / risk_divisor(cx, cy, 0.0)
            if roi > best_roi:
                best_roi = roi
                best_goal = (cx, cy, "CUBE")

        if not has_los_target and d_target_2d > 48.0:
            for p in obs["pickups"]:
                kind = p["kind"]
                if kind in held_kinds:
                    continue
                prio = WEAPON_PRIORITY.get(kind, 0)
                if not need_weapon and prio <= worst_prio:
                    continue
                px, py = p["pos"][0], p["pos"][1]
                reward = (260.0 if need_weapon else 120.0) + prio * 22.0
                roi = reward / risk_divisor(px, py, 0.5)
                if roi > best_roi:
                    best_roi = roi
                    best_goal = (px, py, "LOOT")

        return best_goal if best_roi >= 1.30 else None

    def _local_fight_move(
        self, obs: mf.Observation, target_mech: Dict[str, Any],
        los_mechs: List[Dict[str, Any]], desired_dist: float,
        peek_cover: bool, reload_hide: bool, cur_zc: List[float], cur_zr: float, t_wall0: float,
    ) -> Tuple[float, float]:
        me = obs["self"]
        mx, my, _ = me["pos"]
        eye_h = float(me["stats"].get("eye_height", 8.0))
        tx, ty, _ = target_mech["pos"]
        nc = obs["zone"]["next_center"]
        nr = max(8.0, float(obs["zone"]["next_radius"]))
        ang_from_e = math.atan2(my - ty, mx - tx)
        tang_pref = ang_from_e + self.strafe_dir * 1.57

        t1_scale = 1.0 + self.size_per_cube * float(target_mech.get("cubes", 0.0))
        t1_eye = [tx, ty, target_mech["pos"][2] + 8.0 * t1_scale]
        other_threat = None
        for m2 in los_mechs:
            if m2["id"] != target_mech["id"]:
                other_threat = m2
                break
        if other_threat is None:
            for th in self.threats.values():
                if th["id"] != target_mech["id"] and (obs["t"] - th["t"] < 3.0) and horiz_dist(me["pos"], th["pos"]) < 105.0:
                    other_threat = th
                    break

        t2_eye = None
        if other_threat is not None:
            o_scale = 1.0 + self.size_per_cube * float(other_threat.get("cubes", 0.0))
            op = other_threat["pos"]
            t2_eye = [op[0], op[1], op[2] + 8.0 * o_scale]

        best_dx, best_dy = math.cos(tang_pref), math.sin(tang_pref)
        best_score = -1e9
        do_iso_los = (t2_eye is not None or reload_hide) and ((time.perf_counter() - t_wall0) < 0.011)

        for k in range(16):
            ang = k * (math.pi / 8.0)
            ux, uy = math.cos(ang), math.sin(ang)
            if self.world.point_near_wall(mx + 3.8 * ux, my + 3.8 * uy, 2.6):
                continue
            nx_p, ny_p = mx + 8.5 * ux, my + 8.5 * uy
            if self.world.point_near_wall(nx_p, ny_p, 2.6):
                continue
            if math.hypot(nx_p - cur_zc[0], ny_p - cur_zc[1]) > cur_zr - 9.0:
                continue

            score = 0.0
            d_new = math.hypot(nx_p - tx, ny_p - ty)
            score -= abs(d_new - desired_dist) * 1.15
            score += (ux * math.cos(tang_pref) + uy * math.sin(tang_pref)) * 18.0

            if other_threat is not None:
                d2_new = math.hypot(nx_p - other_threat["pos"][0], ny_p - other_threat["pos"][1])
                d2_old = math.hypot(mx - other_threat["pos"][0], my - other_threat["pos"][1])
                score += (d2_new - d2_old) * 2.0

            if do_iso_los and (k % 4 == 0) and (time.perf_counter() - t_wall0) < 0.015:
                cz = self.world.terrain_z(nx_p, ny_p)
                c_eye = [nx_p, ny_p, cz + eye_h]
                see_t1 = self.world.has_los(c_eye, t1_eye)
                if reload_hide and not see_t1:
                    score += 30.0
                if t2_eye is not None:
                    see_t2 = self.world.has_los(c_eye, t2_eye)
                    if see_t1 and not see_t2:
                        score += 34.0
                    elif see_t1 and see_t2:
                        score -= 22.0

            if peek_cover and self.world.has_near_tall_wall(nx_p, ny_p):
                score += 10.0
            if math.hypot(nx_p - nc[0], ny_p - nc[1]) > nr * 0.88:
                score -= 14.0
            if self.world.point_near_wall(mx + 14.0 * ux, my + 14.0 * uy, 2.4):
                score -= 10.0

            if score > best_score:
                best_score = score
                best_dx, best_dy = ux, uy

        if self.world.point_near_wall(mx + 6.0 * math.cos(tang_pref), my + 6.0 * math.sin(tang_pref), 2.6):
            self.strafe_dir *= -1
            self.last_flip_t = obs["t"]

        return best_dx, best_dy

    def act(self, obs: mf.Observation) -> mf.Action:
        t_wall0 = time.perf_counter()
        t = obs["t"]
        me = obs["self"]
        mx, my, mz = me["pos"]
        mvx, mvy, mvz = me["vel"]
        hp = me["hp"]
        max_hp = me["max_hp"]
        hp_frac = hp / max(1.0, max_hp)
        energy = me["energy"]
        scale = 1.0 + self.size_per_cube * me["cubes"]
        my_eye = [mx, my, mz + float(me["stats"].get("eye_height", 8.0 * scale))]
        my_muzzle_now = [mx, my, mz + float(me["stats"].get("muzzle_height", 7.0 * scale))]
        my_muzzle = [my_muzzle_now[0] + mvx * 0.05, my_muzzle_now[1] + mvy * 0.05, my_muzzle_now[2] + mvz * 0.05]
        my_mortar_muzzle = [mx, my, mz + 9.5 * scale]
        shot_origin_dist = 2.5 * scale + 0.5
        walk_spd = float(me["stats"].get("walk_speed", self.walk_speed))

        if self.last_pos is not None:
            if math.hypot(mx - self.last_pos[0], my - self.last_pos[1]) < 0.14:
                self.stuck_score += 0.1
            else:
                self.stuck_score = max(0.0, self.stuck_score - 0.25)
            if self.stuck_score > 1.0:
                self.unstuck_until = t + 0.55
                self.stuck_score = 0.0
                self.strafe_dir *= -1
                self.last_flip_t = t
        self.last_pos = [mx, my, mz]

        if t >= self.next_strafe_flip_t:
            self.strafe_dir *= -1
            self.last_flip_t = t
            self.next_strafe_flip_t = t + self.rng.uniform(1.00, 1.35)

        act: mf.Action = {"tick": obs["tick"]}

        if obs["offer"]:
            act["pick"] = self._pick_upgrade(obs)

        loot_act = self._choose_loot_action(obs)
        if loot_act:
            act.update(loot_act)  # type: ignore[arg-type]

        rear_bearing = self._update_awareness(obs)

        cur_zc, cur_zr = self._zone_circle(obs)
        z_obs = obs["zone"]
        nc, nr = z_obs["next_center"], max(6.0, float(z_obs["next_radius"]))
        d_cur = math.hypot(mx - cur_zc[0], my - cur_zc[1])
        dist_to_zone_edge = cur_zr - d_cur
        d_next = math.hypot(mx - nc[0], my - nc[1])
        dist_outside_next = max(0.0, d_next - nr * 0.78)
        t_travel_needed = (dist_outside_next * 1.3) / max(1.0, walk_spd)

        if cur_zr > nr + 1.0 and d_cur > nr:
            shrink_dur = max(1.0, z_obs["shrink_end"] - z_obs["shrink_start"])
            frac_left = clamp((cur_zr - d_cur) / max(1.0, cur_zr - nr), 0.0, 1.0)
            t_zone_hits = max(0.0, z_obs["shrink_start"] - t) + shrink_dur * frac_left
        else:
            t_zone_hits = 1e6

        zone_escape = (not me["in_zone"]) or (dist_to_zone_edge < 10.0)
        zone_rotate = (dist_outside_next > 0.0) and (t_zone_hits <= t_travel_needed + 9.0)

        vis_mechs = [m for m in obs["mechs"] if m["kind"] == "mech"]
        vis_npcs = [m for m in obs["mechs"] if m["kind"] == "npc"]
        los_mechs: List[Dict[str, Any]] = [m for m in vis_mechs if self._has_mech_los(t, my_eye, m)]

        target_mech: Optional[Dict[str, Any]] = None
        if los_mechs:
            target_mech = min(
                los_mechs,
                key=lambda m: (
                    horiz_dist(me["pos"], m["pos"])
                    + float(m.get("hp", 1000.0)) * 0.20
                    - (360.0 if float(m.get("hp", 1000.0)) < hp * 0.55 else 0.0)
                ),
            )
        elif vis_mechs:
            target_mech = min(vis_mechs, key=lambda m: horiz_dist(me["pos"], m["pos"]))

        has_los_target = target_mech is not None and (target_mech in los_mechs)
        d_target_2d = horiz_dist(me["pos"], target_mech["pos"]) if target_mech else 999.0
        target_hp = float(target_mech.get("hp", 1000.0)) if target_mech else 9999.0

        kill_confirm = target_mech is not None and target_hp < 550.0 and hp > 280.0 and len(los_mechs) <= 2
        outnumbered = len(los_mechs) >= 2 and hp_frac < 0.55 and target_hp >= 450.0
        losing_duel = (
            target_mech is not None
            and not kill_confirm
            and hp_frac < 0.26
            and target_hp > hp * 1.6
        )
        need_retreat = outnumbered or losing_duel

        held_kinds = {w["kind"] for w in me["weapons"] if w is not None}
        gun_reloading = any(w and w["kind"] == "gun" and float(w.get("reload", 0.0)) > 0.30 for w in me["weapons"])
        special_ready = any(
            w and w["kind"] != "gun" and (float(w.get("cooldown", 0.0)) <= 0.25 or float(w.get("charge", 0.0)) > 0.0)
            for w in me["weapons"]
        )
        reload_cover_dip = gun_reloading and (not special_ready) and (not kill_confirm)

        farm_obj: Optional[Tuple[List[float], List[float], str]] = None
        if not zone_escape and not zone_rotate and target_mech is None and len(los_mechs) == 0 and me["cubes"] < self.cube_stat_cap:
            near_npc = min(vis_npcs, key=lambda n: horiz_dist(me["pos"], n["pos"]), default=None)
            near_crate = min(obs["crates"], key=lambda c: horiz_dist(me["pos"], c["pos"]), default=None)
            if near_npc and horiz_dist(me["pos"], near_npc["pos"]) < 95.0 and hp_frac > 0.58:
                np_p = near_npc["pos"]
                if self.world.has_los(my_eye, [np_p[0], np_p[1], np_p[2] + 1.75]):
                    farm_obj = ([np_p[0], np_p[1], np_p[2] + 1.75], list(near_npc["vel"]), "npc")
            elif near_crate and horiz_dist(me["pos"], near_crate["pos"]) < 85.0 and hp_frac > 0.60:
                cp = near_crate["pos"]
                if self.world.has_los(my_eye, [cp[0], cp[1], cp[2] + 1.0]):
                    farm_obj = ([cp[0], cp[1], cp[2] + 1.0], [0.0, 0.0, 0.0], "crate")

        goal_x, goal_y = self._safe_anchor(obs)
        want_boost = False
        mode = "PATROL"
        use_local_fight_move = False

        loot_goal = None
        if not zone_escape and not need_retreat and not zone_rotate:
            loot_goal = self._eval_loot_target(obs, cur_zc, cur_zr, has_los_target, d_target_2d)

        if zone_escape:
            mode = "ZONE_ESC"
            goal_x, goal_y = self._safe_anchor(obs)
            want_boost = (not me["in_zone"] and energy > 15.0) or (dist_to_zone_edge < 6.0 and energy > 25.0)

        elif need_retreat:
            mode = "RETREAT"
            goal_x, goal_y = self._find_cover_pos(obs)
            if energy > 22.0:
                want_boost = True

        elif zone_rotate:
            mode = "ROTATE"
            goal_x, goal_y = self._safe_anchor(obs)

        elif loot_goal is not None and not kill_confirm and not (has_los_target and d_target_2d < 70.0 and loot_goal[2] != "CUBE"):
            goal_x, goal_y, mode = loot_goal

        elif target_mech is not None:
            mode = "KILL" if kill_confirm else ("RELOAD" if reload_cover_dip else "FIGHT")
            e_weapons = set(target_mech.get("weapons", []))

            # v9 sweet-spot range (40m 1v1, 54m 2v1/reload-dip, 32m kill-confirm)
            desired_dist = 54.0 if (len(los_mechs) >= 2 or reload_cover_dip) else 40.0
            if kill_confirm:
                desired_dist = 32.0
                if (not has_los_target or d_target_2d > 45.0) and energy > 45.0:
                    want_boost = True
            elif "lightning" in e_weapons and "lightning" not in held_kinds:
                desired_dist = 72.0
            elif "lightning" in held_kinds and target_mech.get("charging") and energy > 38.0 and d_target_2d < 65.0:
                desired_dist = 32.0
            elif "mortar" in held_kinds and "laser" not in held_kinds and not has_los_target:
                desired_dist = 80.0

            if has_los_target or d_target_2d < 70.0:
                use_local_fight_move = True
                move_x, move_y = self._local_fight_move(
                    obs, target_mech, los_mechs, desired_dist,
                    reload_cover_dip or hp_frac < 0.48, reload_cover_dip,
                    cur_zc, cur_zr, t_wall0,
                )
            else:
                goal_x, goal_y = target_mech["pos"][0], target_mech["pos"][1]

            # Strafe-flip boost burst (breaks enemy lead right on direction change, reserves >=40 energy)
            if has_los_target and energy > 65.0 and ((t - self.last_flip_t) < 0.25 or reload_cover_dip):
                want_boost = True

        elif farm_obj is not None:
            mode = "FARM"
            fx, fy, _ = farm_obj[0]
            ang_from_f = math.atan2(my - fy, mx - fx)
            tang = ang_from_f + self.strafe_dir * 0.65
            dist_f = 28.0 if farm_obj[2] == "npc" else 18.0
            goal_x, goal_y = fx + dist_f * math.cos(tang), fy + dist_f * math.sin(tang)

        else:
            mode = "PATROL"
            patrol_candidates: List[Tuple[float, float, float]] = []
            for c in obs["cubes"]:
                cx, cy = float(c["pos"][0]), float(c["pos"][1])
                d_zone = math.hypot(cx - nc[0], cy - nc[1])
                d_me = math.hypot(cx - mx, cy - my)
                if d_zone <= nr * 0.92:
                    patrol_candidates.append((d_me, cx, cy))
            if patrol_candidates:
                _, goal_x, goal_y = min(patrol_candidates)
            else:
                ang_patrol = t * 0.35
                pat_r = min(nr * 0.52, max(8.0, nr - 20.0))
                goal_x = nc[0] + pat_r * math.cos(ang_patrol)
                goal_y = nc[1] + pat_r * math.sin(ang_patrol)

        if not use_local_fight_move:
            wp_reached = self.cached_wp is not None and math.hypot(self.cached_wp[0] - mx, self.cached_wp[1] - my) < 3.5
            need_path = (
                self.cached_wp is None
                or self.cached_goal is None
                or wp_reached
                or math.hypot(goal_x - self.cached_goal[0], goal_y - self.cached_goal[1]) > 14.0
                or t - self.last_path_t > 0.7
            )
            if need_path and (time.perf_counter() - t_wall0) < 0.015:
                self.cached_goal = (goal_x, goal_y)
                self.cached_wp = self.world.find_next_waypoint(mx, my, goal_x, goal_y)
                self.last_path_t = t

            wx, wy = self.cached_wp if self.cached_wp is not None else (goal_x, goal_y)
            move_x, move_y = wx - mx, wy - my
            mlen = math.hypot(move_x, move_y)
            if mlen > 1.2:
                move_x /= mlen
                move_y /= mlen
            elif mode in ("CUBE", "LOOT"):
                move_x /= 1.2
                move_y /= 1.2
            else:
                move_x, move_y = 0.0, 0.0

        dx_d, dy_d, dodge_boost, dodge_jump = self._dodge_vector(obs)
        if dx_d != 0.0 or dy_d != 0.0:
            move_x += dx_d
            move_y += dy_d
            if dodge_boost and energy > 22.0:
                want_boost = True

        want_jump = dodge_jump
        if t < self.unstuck_until:
            px, py = -move_y, move_x
            if px == 0.0 and py == 0.0:
                px, py = math.cos(t), math.sin(t)
            move_x = 0.4 * move_x + 0.9 * self.strafe_dir * px
            move_y = 0.4 * move_y + 0.9 * self.strafe_dir * py
            if me["grounded"]:
                want_jump = True
            elif energy > 25.0:
                want_boost = True

        if energy < 20.0 and not (not me["in_zone"] and energy > 8.0):
            want_boost = False

        if me["in_zone"] and dist_to_zone_edge < 8.0:
            to_zc_x, to_zc_y = cur_zc[0] - mx, cur_zc[1] - my
            zd = math.hypot(to_zc_x, to_zc_y) or 1.0
            if move_x * to_zc_x + move_y * to_zc_y < 0.0:
                move_x, move_y = to_zc_x / zd, to_zc_y / zd

        mlen = math.hypot(move_x, move_y)
        if mlen > 1.0:
            move_x /= mlen
            move_y /= mlen

        act["move"] = [round(move_x, 4), round(move_y, 4)]
        act["boost"] = want_boost
        act["jump"] = want_jump

        # --- Aiming & Firing -----------------------------------------------
        fire = [False, False]
        want_reload = False
        aim_yaw = me["aim"][0]
        aim_pitch = 0.0

        if target_mech is not None:
            t_pos = list(target_mech["pos"])
            t_vel = list(target_mech["vel"])
            t_scale = 1.0 + self.size_per_cube * float(target_mech.get("cubes", 0.0))
            t_com = [
                t_pos[0] + t_vel[0] * 0.05,
                t_pos[1] + t_vel[1] * 0.05,
                t_pos[2] + t_vel[2] * 0.05 + 4.5 * t_scale,
            ]
            t_laser_pt = [
                t_pos[0] + t_vel[0] * 0.025,
                t_pos[1] + t_vel[1] * 0.025,
                t_pos[2] + t_vel[2] * 0.025 + 4.5 * t_scale,
            ]
            d_to_target = dist3(my_muzzle, t_com)

            direct_yaw = math.atan2(t_laser_pt[1] - my_muzzle[1], t_laser_pt[0] - my_muzzle[0])
            direct_pitch = math.atan2(t_laser_pt[2] - my_muzzle[2], max(0.1, horiz_dist(my_muzzle, t_laser_pt)))
            aim_yaw, aim_pitch = direct_yaw, direct_pitch

            if rear_bearing is not None and not has_los_target:
                aim_yaw, aim_pitch = rear_bearing, 0.0

            allow_fire = has_los_target or not need_retreat

            if allow_fire:
                aim_owner: Optional[int] = None
                mortar_sol: Optional[Tuple[float, float]] = None
                gun_ready = any(w and w["kind"] == "gun" and float(w.get("reload", 0.0)) <= 0.10 and w.get("ammo", 0) > 0 for w in me["weapons"])

                for idx, w in enumerate(me["weapons"]):
                    if w and w["kind"] == "laser":
                        l_range = float(w["stats"].get("range", 280.0))
                        ch = float(w.get("charge", 0.0))
                        full_c = float(w["stats"].get("charge", 0.75))
                        min_c = float(w["stats"].get("min_charge", 0.15))
                        min_frac = float(w["stats"].get("min_frac", 0.35))
                        dmg_full = float(w["stats"].get("damage", 315.0))
                        if "mortar" in target_mech.get("weapons", []):
                            dmg_full *= 1.6
                        est_dmg = dmg_full * (min_frac + (1.0 - min_frac) * clamp(ch / max(0.01, full_c), 0.0, 1.0))
                        snap_now = (ch >= full_c - 0.24) or (ch >= min_c and est_dmg >= target_hp * 1.02)
                        if (ch > 0.0 and (snap_now or not gun_ready)) and d_to_target <= l_range:
                            aim_owner = idx
                            break

                if aim_owner is None:
                    for idx, w in enumerate(me["weapons"]):
                        if w and w["kind"] == "mortar" and float(w["cooldown"]) <= 0.10:
                            if (not has_los_target or gun_reloading or d_to_target >= 55.0) and (time.perf_counter() - t_wall0) < 0.020:
                                t_acc = float(self.threats.get(int(target_mech["id"]), {}).get("accel", 0.0))
                                mortar_sol = solve_mortar_aim(
                                    my_mortar_muzzle, t_pos, t_vel,
                                    float(w["stats"].get("speed", 72.0)),
                                    float(w["stats"].get("gravity", 22.0)),
                                    self.world,
                                    shot_origin_dist=shot_origin_dist,
                                    lead_scale=0.58 if t_acc > 20.0 else 0.85,
                                    prefer_high=not has_los_target,
                                )
                                if mortar_sol is not None:
                                    aim_owner = idx
                                    break

                if aim_owner is not None:
                    if me["weapons"][aim_owner]["kind"] == "mortar" and mortar_sol is not None:  # type: ignore[index]
                        aim_yaw, aim_pitch = mortar_sol
                    else:
                        aim_yaw, aim_pitch = direct_yaw, direct_pitch
                else:
                    for idx, w in enumerate(me["weapons"]):
                        if w and w["kind"] == "gun" and float(w.get("reload", 0.0)) <= 0.10 and w.get("ammo", 0) > 0 and has_los_target:
                            aim_yaw, aim_pitch = solve_lead_aim(my_muzzle, t_com, t_vel, float(w["stats"].get("speed", 252.0)))
                            break

                for slot_i, w in enumerate(me["weapons"]):
                    if w is None:
                        continue
                    kind = w["kind"]
                    st = w["stats"]
                    cd = float(w.get("cooldown", 0.0))

                    if kind == "laser":
                        l_range = float(st.get("range", 280.0))
                        charge = float(w.get("charge", 0.0))
                        full_c = float(st.get("charge", 0.75))
                        min_c = float(st.get("min_charge", 0.15))
                        min_frac = float(st.get("min_frac", 0.35))
                        dmg_full = float(st.get("damage", 315.0))
                        if "mortar" in target_mech.get("weapons", []):
                            dmg_full *= 1.6
                        err_y = angle_diff(me["aim"][0], direct_yaw)
                        err_p = abs(me["aim"][1] - direct_pitch)
                        tol = clamp(math.atan2(1.7 * t_scale, max(1.0, d_to_target)), 0.024, 0.085)
                        if charge > 0.0:
                            frac = clamp(charge / max(0.01, full_c), 0.0, 1.0)
                            est_dmg = dmg_full * (min_frac + (1.0 - min_frac) * frac)
                            ready_rel = charge >= full_c - 0.08 or (charge >= min_c and est_dmg >= target_hp * 1.02)
                            if ready_rel and has_los_target and (err_y < tol or charge >= full_c + 0.35) and err_p < tol * 1.6:
                                fire[slot_i] = False
                            else:
                                fire[slot_i] = True
                        elif cd <= 0.10 and d_to_target <= l_range and has_los_target and err_y < 0.45:
                            if not (hp_frac < 0.32 and d_to_target < 38.0):
                                fire[slot_i] = True

                    elif kind == "mortar":
                        if mortar_sol is not None and cd <= 0.10:
                            m_yaw, m_pitch = mortar_sol
                            if angle_diff(me["aim"][0], m_yaw) < 0.14 and abs(me["aim"][1] - m_pitch) < 0.14:
                                fire[slot_i] = True

                    elif kind == "lightning":
                        lt_range = float(st.get("range", 45.0))
                        cone_rad = math.radians(float(st.get("cone_deg", 30.0)) * 0.5)
                        if cd <= 0.10 and d_to_target <= lt_range and has_los_target and energy > 20.0:
                            if angle_diff(me["aim"][0], direct_yaw) <= cone_rad * 0.95:
                                fire[slot_i] = True

                    elif kind == "gun":
                        g_range = float(st.get("range", 230.0))
                        g_speed = float(st.get("speed", 252.0))
                        ammo = float(w.get("ammo", 0.0))
                        reloading = float(w.get("reload", 0.0)) > 0.10
                        mag = float(st.get("mag", 12.0))
                        if (
                            ammo <= 1
                            or (ammo <= mag * 0.20 and (reload_cover_dip or not has_los_target))
                        ) and float(w.get("reload", 0.0)) <= 0.0:
                            want_reload = True
                        if not reloading and ammo > 0 and d_to_target <= g_range and has_los_target:
                            g_yaw, g_pitch = solve_lead_aim(my_muzzle, t_com, t_vel, g_speed)
                            err_y = angle_diff(me["aim"][0], g_yaw)
                            err_p = abs(me["aim"][1] - g_pitch)
                            if cd <= 0.10 and err_y < 0.22 and err_p < 0.22:
                                fire[slot_i] = True

        elif farm_obj is not None:
            ft_pos, ft_vel, _ = farm_obj
            d_ft = dist3(my_muzzle, ft_pos)
            for slot_i, w in enumerate(me["weapons"]):
                if w is None:
                    continue
                kind = w["kind"]
                if kind == "gun" and float(w.get("reload", 0.0)) <= 0.10 and w.get("ammo", 0) > 0 and d_ft <= float(w["stats"].get("range", 230.0)):
                    aim_yaw, aim_pitch = solve_lead_aim(my_muzzle, ft_pos, ft_vel, float(w["stats"].get("speed", 252.0)))
                    if float(w["cooldown"]) <= 0.10 and angle_diff(me["aim"][0], aim_yaw) < 0.18:
                        fire[slot_i] = True
                elif kind == "laser" and d_ft <= 130.0:
                    ch = float(w.get("charge", 0.0))
                    full_c = float(w["stats"].get("charge", 0.75))
                    ly = math.atan2(ft_pos[1] - my_muzzle[1], ft_pos[0] - my_muzzle[0])
                    lp = math.atan2(ft_pos[2] - my_muzzle[2], max(0.1, horiz_dist(my_muzzle, ft_pos)))
                    if ch > 0.0:
                        if ch >= full_c - 0.08 and angle_diff(me["aim"][0], ly) < 0.06:
                            fire[slot_i] = False
                        else:
                            fire[slot_i] = True
                    elif float(w["cooldown"]) <= 0.10 and angle_diff(me["aim"][0], ly) < 0.25:
                        fire[slot_i] = True

        else:
            if rear_bearing is not None:
                aim_yaw, aim_pitch = rear_bearing, 0.0
            else:
                recent = sorted((th for th in self.threats.values() if t - th["t"] < 4.5), key=lambda th: t - th["t"], reverse=True)
                if recent:
                    th = recent[0]
                    aim_yaw = math.atan2(th["pos"][1] - my_muzzle[1], th["pos"][0] - my_muzzle[0])
                elif self.recent_sounds:
                    aim_yaw = self.recent_sounds[-1][1]
                else:
                    base_yaw = math.atan2(move_y, move_x) if (move_x != 0.0 or move_y != 0.0) else me["aim"][0]
                    aim_yaw = wrap_angle(base_yaw + 1.15 * math.sin(t * 1.5))

            for w in me["weapons"]:
                if w and w["kind"] == "gun" and w.get("reload", 0.0) <= 0.0:
                    if float(w["ammo"]) < float(w["stats"].get("mag", 12.0)) * 0.65:
                        want_reload = True

        act["aim"] = [round(wrap_angle(aim_yaw), 4), round(clamp(aim_pitch, self.min_pitch, self.max_pitch), 4)]
        act["fire"] = fire
        if want_reload:
            act["reload"] = True

        mf.draw.point([goal_x, goal_y, mz], "#40c0ff", 5)
        mf.draw.label(f"{mode} | hp {hp:.0f} | c {me['cubes']:.0f}")
        return act


if __name__ == "__main__":
    mf.run(Bot)