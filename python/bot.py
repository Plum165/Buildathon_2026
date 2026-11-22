"""Fast, survival + combat bot for mechfight/1 (standard library only)."""
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
# Map geometry, terrain, 3D line-of-sight, and fast A* navigation
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

        self.bucket_size = 20.0
        self.buckets: Dict[Tuple[int, int], List[int]] = {}

        self.nav_step = 5.0
        self.nav_nx = 0
        self.nav_ny = 0
        self.nav_min_x = 0.0
        self.nav_min_y = 0.0
        self.nav_blocked = bytearray()
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

        self._build_nav_grid(mech_cfg)
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

    def _build_nav_grid(self, mech_cfg: Dict[str, float]) -> None:
        step_up = float(mech_cfg.get("step_up", 1.2))
        mech_h = float(mech_cfg.get("height", 9.0))
        max_slope_tan = math.tan(math.radians(float(mech_cfg.get("max_slope_deg", 50.0)) - 3.0))

        self.nav_min_x = float(self.bounds["minX"])
        self.nav_min_y = float(self.bounds["minY"])
        self.nav_nx = int((self.bounds["maxX"] - self.nav_min_x) / self.nav_step) + 1
        self.nav_ny = int((self.bounds["maxY"] - self.nav_min_y) / self.nav_step) + 1
        total = self.nav_nx * self.nav_ny
        blocked = bytearray(total)

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
                    if max(dzx, dzy) > max_slope_tan:
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
                    if blocked[row + gx]:
                        continue
                    x = min_x + (gx + 0.5) * s
                    tz = self.terrain_z(x, y)
                    if (p.base - tz) >= mech_h or (p.top - tz) <= step_up:
                        continue
                    if _point_in_prism_2d(x, y, p):
                        blocked[row + gx] = 1
                        continue
                    for ax, ay, bx, by in p.edges:
                        if _dist_point_to_seg_sq(x, y, ax, ay, bx, by) < clear_r_sq:
                            blocked[row + gx] = 1
                            break

        self.nav_blocked = blocked

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
                penalty[idx] = 2.2
            elif d == 2:
                penalty[idx] = 0.5
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
        """Fast 3D LOS check against prisms and terrain."""
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
                if z1 + dz * t < self.terrain_z(x1 + dx * t, y1 + dy * t) - 0.1:
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
    speed: float, max_time: float = 0.85,
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
    yaw = math.atan2(py, px)
    pitch = math.atan2(pz, max(1e-4, math.hypot(px, py)))
    return yaw, pitch


def solve_mortar_aim(
    muzzle: List[float], target_pos: List[float], target_vel: List[float],
    speed: float, gravity: float, world_map: WorldMap,
) -> Optional[Tuple[float, float]]:
    pred = [target_pos[0], target_pos[1], target_pos[2]]
    flight_t = horiz_dist(muzzle, pred) / max(speed * 0.8, 1.0)
    for _ in range(3):
        lt = clamp(flight_t, 0.0, 2.8)
        pred = [
            target_pos[0] + target_vel[0] * lt * 0.7,
            target_pos[1] + target_vel[1] * lt * 0.7,
            target_pos[2] + target_vel[2] * lt * 0.4,
        ]
        dx = pred[0] - muzzle[0]
        dy = pred[1] - muzzle[1]
        dz = pred[2] - muzzle[2]
        r = math.hypot(dx, dy)
        if r < 15.0:
            return None
        v2 = speed * speed
        disc = v2 * v2 - gravity * (gravity * r * r + 2.0 * dz * v2)
        if disc < 0.0:
            return None
        sq = math.sqrt(disc)
        pitch_low = math.atan2(v2 - sq, gravity * r)
        vx_h = speed * math.cos(pitch_low)
        if vx_h > 1.0:
            flight_t = r / vx_h

    dx = pred[0] - muzzle[0]
    dy = pred[1] - muzzle[1]
    dz = pred[2] - muzzle[2]
    r = math.hypot(dx, dy)
    if r < 15.0:
        return None
    v2 = speed * speed
    disc = v2 * v2 - gravity * (gravity * r * r + 2.0 * dz * v2)
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    yaw = math.atan2(dy, dx)

    for pitch in (math.atan2(v2 - sq, gravity * r), math.atan2(v2 + sq, gravity * r)):
        if not (-1.2 <= pitch <= 1.35):
            continue
        cos_p = math.cos(pitch)
        sin_p = math.sin(pitch)
        if cos_p < 0.05:
            continue
        total_t = r / (speed * cos_p)
        vx = speed * cos_p * math.cos(yaw)
        vy = speed * cos_p * math.sin(yaw)
        vz = speed * sin_p
        ox = muzzle[0] + 3.0 * cos_p * math.cos(yaw)
        oy = muzzle[1] + 3.0 * cos_p * math.sin(yaw)
        oz = muzzle[2] + 3.0 * sin_p
        ok = True
        prev = [ox, oy, oz]
        for k in range(1, 8):
            st = (k / 8.0) * total_t * 0.85
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
    name = "survivor-v2"

    def __init__(self, hello: mf.Hello) -> None:
        self.cfg = hello["config"]
        self.rng = random.Random(int(hello.get("seed", 42)))
        self.pickup_radius = float(self.cfg["loot"]["pickup_radius"])
        self.mech_cfg = self.cfg["mech"]
        self.min_pitch = float(self.mech_cfg.get("min_pitch", -1.2))
        self.max_pitch = float(self.mech_cfg.get("max_pitch", 1.2))
        self.regen_delay = float(self.mech_cfg.get("regen_delay", 5.0))
        self.size_per_cube = float(self.cfg.get("cubes", {}).get("size_per_cube", 0.05))
        self.upgrades_def: Dict[str, Dict[str, Any]] = {
            u["id"]: u for u in self.cfg.get("upgrades", [])
        }

        map_dir = hello.get("map", {}).get("dir", "/arena")
        self.world = WorldMap(map_dir, self.mech_cfg)

        self.last_combat_t = -10.0
        self.threats: Dict[int, Dict[str, Any]] = {}
        self.recent_sounds: List[Tuple[float, float, str]] = []
        self.strafe_dir = 1 if self.rng.random() < 0.5 else -1
        self.last_strafe_flip_t = 0.0

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
        held_kinds = {w["kind"] for w in obs["self"]["weapons"] if w is not None}
        best_i = 0
        best_score = -1e9
        for idx, choice in enumerate(offer["choices"]):
            uid = choice["id"]
            udef = self.upgrades_def.get(uid)
            score = 20.0
            if udef:
                req = udef.get("requires")
                if req and req not in held_kinds:
                    score -= 50.0
                for eff in udef.get("effects", []):
                    path = eff.get("path", "")
                    mul = float(eff.get("mul", 1.0))
                    add = float(eff.get("add", 0.0))
                    if "damage_mult" in path or "armor" in path:
                        score += 110.0
                    elif "regen_per_s" in path or "max_hp" in path or "regen_delay" in path:
                        score += 95.0
                    elif req in held_kinds or any(path.startswith(k) for k in held_kinds):
                        score += 88.0
                    elif "speed" in path or "energy" in path or "jet" in path:
                        score += 80.0
                    elif "near_range" in path or "bearing_noise" in path or "vision" in path:
                        score += 75.0
                    elif "turn_rate" in path or "pitch_rate" in path:
                        score += 55.0
                    else:
                        score += abs(mul - 1.0) * 40.0 + abs(add) * 2.0
            else:
                desc = (choice.get("name", "") + " " + choice.get("desc", "")).lower()
                if "armor" in desc or "damage taken" in desc or "hp" in desc or "regen" in desc:
                    score += 95.0
                elif "speed" in desc or "energy" in desc or "fire rate" in desc or "damage" in desc:
                    score += 80.0
            if score > best_score:
                best_score = score
                best_i = idx
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
        combat_now = False
        rear_bearing: Optional[float] = None

        for ev in obs["events"]:
            et = ev["type"]
            if et == "damaged":
                combat_now = True
                if ev.get("rear") and ev.get("bearing") is not None:
                    rear_bearing = float(ev["bearing"])
                by = ev.get("by")
                if by is not None and ev.get("bearing") is not None:
                    bid = int(by)
                    if bid not in self.threats:
                        b = float(ev["bearing"])
                        self.threats[bid] = {
                            "id": bid,
                            "pos": [mx + 45.0 * math.cos(b), my + 45.0 * math.sin(b), mz],
                            "vel": [0.0, 0.0, 0.0],
                            "aim": [b + math.pi, 0.0],
                            "hp": 1400.0,
                            "weapons": [],
                            "charging": False,
                            "cubes": 0.0,
                            "t": t,
                            "visible": False,
                        }
            elif et == "hit":
                combat_now = True

        if combat_now:
            self.last_combat_t = t

        visible_ids: Set[int] = set()
        for m in obs["mechs"]:
            if m["kind"] != "mech":
                continue
            mid = int(m["id"])
            visible_ids.add(mid)
            self.threats[mid] = {
                "id": mid,
                "pos": list(m["pos"]),
                "vel": list(m["vel"]),
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
                self.recent_sounds.append((t - float(s["age"]), float(s["bearing"]), s["dist"]))

        return rear_bearing

    def _dodge_vector(self, obs: mf.Observation) -> Tuple[float, float, bool]:
        me = obs["self"]
        mx, my, mz = me["pos"]
        scale = 1.0 + self.size_per_cube * me["cubes"]
        com_z = mz + 4.5 * scale
        dodge_x, dodge_y = 0.0, 0.0
        urgent_boost = False
        g = float(self.cfg["weapons"]["mortar"].get("gravity", 22.0))

        for p in obs["projectiles"][:16]:
            px, py, pz = p["pos"]
            vx, vy, vz = p["vel"]
            if p["kind"] == "mortar":
                for step_i in range(1, 25):
                    st = step_i * 0.08
                    sx = px + vx * st
                    sy = py + vy * st
                    sz = pz + vz * st - 0.5 * g * st * st
                    if sz <= mz + 1.0:
                        d_imp = math.hypot(mx - sx, my - sy)
                        if d_imp < 9.0:
                            away_x, away_y = mx - sx, my - sy
                            ln = math.hypot(away_x, away_y) or 1.0
                            w = (9.5 - d_imp) * 0.45
                            dodge_x += (away_x / ln) * w
                            dodge_y += (away_y / ln) * w
                            if st < 0.8 and d_imp < 7.0:
                                urgent_boost = True
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
                    if d_ca < 4.6 * scale:
                        hv = math.hypot(vx, vy) or 1.0
                        px_n, py_n = -vy / hv, vx / hv
                        sign = 1.0 if ((mx - cx) * px_n + (my - cy) * py_n) >= 0.0 else -1.0
                        w = (4.8 * scale - d_ca) * 0.55
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
            if 0.0 < proj < 260.0:
                perp_x, perp_y = rx - proj * ux, ry - proj * uy
                d_perp = math.hypot(perp_x, perp_y)
                if d_perp < 6.5:
                    if d_perp > 0.1:
                        dodge_x += (perp_x / d_perp) * 1.4
                        dodge_y += (perp_y / d_perp) * 1.4
                    else:
                        dodge_x += -uy * 1.4
                        dodge_y += ux * 1.4
                    urgent_boost = True

        return dodge_x, dodge_y, urgent_boost

    def _find_cover_pos(self, obs: mf.Observation) -> Tuple[float, float]:
        t = obs["t"]
        if self.cached_cover is not None and t - self.last_cover_t < 1.0:
            return self.cached_cover

        me = obs["self"]
        mx, my, _ = me["pos"]
        eye_h = float(me["stats"].get("eye_height", 8.0))
        nc = obs["zone"]["next_center"]
        nr = float(obs["zone"]["next_radius"])

        active_threats = sorted(
            (th for th in self.threats.values() if t - th["t"] < 4.5),
            key=lambda th: math.hypot(th["pos"][0] - mx, th["pos"][1] - my),
        )[:2]

        best_pos = (nc[0], nc[1])
        best_score = -1e9
        for dist_r in (18.0, 34.0):
            for k in range(8):
                ang = k * (math.pi / 4.0)
                cx = mx + dist_r * math.cos(ang)
                cy = my + dist_r * math.sin(ang)
                if self.world.is_blocked(cx, cy):
                    continue
                cz = self.world.terrain_z(cx, cy)
                c_eye = [cx, cy, cz + eye_h]
                score = 0.0
                d_next = math.hypot(cx - nc[0], cy - nc[1])
                score += 60.0 if d_next <= max(10.0, nr * 0.9) else -(d_next - nr) * 1.5
                for th in active_threats:
                    tx, ty, tz = th["pos"]
                    t_scale = 1.0 + self.size_per_cube * th.get("cubes", 0.0)
                    if not self.world.has_los(c_eye, [tx, ty, tz + 8.0 * t_scale]):
                        score += 120.0
                    score += min(math.hypot(cx - tx, cy - ty), 90.0) * 0.4
                score -= dist_r * 0.5
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
        # Stay inside safe next_radius without needlessly charging the dead center
        max_r = max(4.0, nr * 0.80)
        if d <= max_r:
            return mx, my
        ang = math.atan2(dy, dx) if d > 1.0 else 0.0
        target_r = min(nr * 0.65, max(0.0, nr - 15.0))
        return nc[0] + target_r * math.cos(ang), nc[1] + target_r * math.sin(ang)

    def _has_mech_los(self, my_eye: List[float], m: Dict[str, Any]) -> bool:
        t_scale = 1.0 + self.size_per_cube * float(m.get("cubes", 0.0))
        mp = m["pos"]
        return (
            self.world.has_los(my_eye, [mp[0], mp[1], mp[2] + 4.5 * t_scale])
            or self.world.has_los(my_eye, [mp[0], mp[1], mp[2] + 8.2 * t_scale])
        )

    def act(self, obs: mf.Observation) -> mf.Action:
        t_wall0 = time.perf_counter()
        t = obs["t"]
        me = obs["self"]
        mx, my, mz = me["pos"]
        hp = me["hp"]
        max_hp = me["max_hp"]
        hp_frac = hp / max(1.0, max_hp)
        energy = me["energy"]
        scale = 1.0 + self.size_per_cube * me["cubes"]
        my_eye = [mx, my, mz + float(me["stats"].get("eye_height", 8.0 * scale))]
        my_muzzle = [mx, my, mz + float(me["stats"].get("muzzle_height", 7.0 * scale))]
        my_mortar_muzzle = [mx, my, mz + 9.5 * scale]

        # Stuck detection
        if self.last_pos is not None:
            if math.hypot(mx - self.last_pos[0], my - self.last_pos[1]) < 0.14:
                self.stuck_score += 0.1
            else:
                self.stuck_score = max(0.0, self.stuck_score - 0.25)
            if self.stuck_score > 1.0:
                self.unstuck_until = t + 0.6
                self.stuck_score = 0.0
                self.strafe_dir *= -1
        self.last_pos = [mx, my, mz]

        if t - self.last_strafe_flip_t > 1.05:
            self.strafe_dir *= -1
            self.last_strafe_flip_t = t

        act: mf.Action = {"tick": obs["tick"]}

        if obs["offer"]:
            act["pick"] = self._pick_upgrade(obs)

        loot_act = self._choose_loot_action(obs)
        if loot_act:
            act.update(loot_act)  # type: ignore[arg-type]

        rear_bearing = self._update_awareness(obs)

        cur_zc, cur_zr = self._zone_circle(obs)
        dist_to_zone_edge = cur_zr - math.hypot(mx - cur_zc[0], my - cur_zc[1])
        zone_danger = (not me["in_zone"]) or (dist_to_zone_edge < 14.0 and t >= obs["zone"]["shrink_start"] - 4.0)

        vis_mechs = [m for m in obs["mechs"] if m["kind"] == "mech"]
        vis_npcs = [m for m in obs["mechs"] if m["kind"] == "npc"]

        # Check which mechs are actually in LOS (not behind cover via 5s memory_s)
        los_mechs: List[Dict[str, Any]] = []
        for m in vis_mechs:
            if self._has_mech_los(my_eye, m):
                los_mechs.append(m)

        # Pick primary target (prefer enemies in LOS, lowest HP / closest)
        target_mech: Optional[Dict[str, Any]] = None
        if los_mechs:
            target_mech = min(
                los_mechs,
                key=lambda m: horiz_dist(me["pos"], m["pos"]) + float(m.get("hp", 1000.0)) * 0.08,
            )
        elif vis_mechs:
            target_mech = min(vis_mechs, key=lambda m: horiz_dist(me["pos"], m["pos"]))

        has_los_target = target_mech is not None and (target_mech in los_mechs)

        # Only enter RECOVER (hold fire for 5s regen) when we are SAFE from LOS or outnumbered & hurt!
        # NEVER stop shooting when in a 1v1 with clear LOS!
        outnumbered = len(los_mechs) >= 2 and hp_frac < 0.72
        losing_duel = (
            target_mech is not None
            and hp_frac < 0.40
            and float(target_mech.get("hp", 1400.0)) > hp * 1.4
        )
        safe_regen_hold = (len(los_mechs) == 0 and hp_frac < 0.78)
        recovering = not zone_danger and (outnumbered or losing_duel or safe_regen_hold)

        # --- Navigation goal selection -------------------------------------
        goal_x, goal_y = self._safe_anchor(obs)
        want_boost = False
        mode = "HOLD"

        held_kinds = {w["kind"] for w in me["weapons"] if w is not None}
        need_weapon = len(held_kinds) < 2

        if zone_danger:
            mode = "ZONE"
            goal_x, goal_y = self._safe_anchor(obs)
            want_boost = (not me["in_zone"] and energy > 20.0) or (dist_to_zone_edge < 6.0 and energy > 35.0)
        elif outnumbered or losing_duel:
            mode = "RETREAT"
            goal_x, goal_y = self._find_cover_pos(obs)
            if energy > 28.0:
                want_boost = True
        else:
            best_cube = min(obs["cubes"], key=lambda c: horiz_dist(me["pos"], c["pos"]), default=None)
            avail_pickups = [
                p for p in obs["pickups"]
                if p["kind"] not in held_kinds and (
                    need_weapon or WEAPON_PRIORITY.get(p["kind"], 0) > min(WEAPON_PRIORITY.get(k, 0) for k in held_kinds)
                )
            ]
            best_pickup = min(avail_pickups, key=lambda p: horiz_dist(me["pos"], p["pos"]), default=None)

            if best_cube and horiz_dist(me["pos"], best_cube["pos"]) < 50.0:
                mode = "CUBE"
                goal_x, goal_y = best_cube["pos"][0], best_cube["pos"][1]
            elif best_pickup and horiz_dist(me["pos"], best_pickup["pos"]) < 55.0 and len(los_mechs) == 0:
                mode = "LOOT"
                goal_x, goal_y = best_pickup["pos"][0], best_pickup["pos"][1]
            elif target_mech is not None:
                mode = "FIGHT"
                tx, ty, _ = target_mech["pos"]
                e_weapons = set(target_mech.get("weapons", []))
                desired_dist = 62.0
                if "lightning" in e_weapons and "lightning" not in held_kinds:
                    desired_dist = 78.0
                elif "lightning" in held_kinds and (target_mech.get("charging") or "laser" in e_weapons) and energy > 40.0 and hp_frac > 0.65:
                    desired_dist = 32.0
                elif "mortar" in held_kinds and "laser" not in held_kinds and not has_los_target:
                    desired_dist = 95.0

                ang_from_e = math.atan2(my - ty, mx - tx)
                # Wall-aware strafe check
                perp_ang = ang_from_e + self.strafe_dir * 1.57
                if self.world.is_blocked(mx + 6.0 * math.cos(perp_ang), my + 6.0 * math.sin(perp_ang)):
                    self.strafe_dir *= -1
                tang_ang = ang_from_e + self.strafe_dir * 0.65
                gx_f = tx + desired_dist * math.cos(tang_ang)
                gy_f = ty + desired_dist * math.sin(tang_ang)
                nc = obs["zone"]["next_center"]
                nr = max(10.0, float(obs["zone"]["next_radius"]))
                if math.hypot(gx_f - nc[0], gy_f - nc[1]) > nr * 0.88:
                    gx_f = 0.6 * gx_f + 0.4 * nc[0]
                    gy_f = 0.6 * gy_f + 0.4 * nc[1]
                goal_x, goal_y = gx_f, gy_f

        # --- Fast Pathfinding + Dodge blending -----------------------------
        need_path = (
            self.cached_wp is None
            or self.cached_goal is None
            or math.hypot(goal_x - self.cached_goal[0], goal_y - self.cached_goal[1]) > 14.0
            or t - self.last_path_t > 0.8
        )
        if need_path and (time.perf_counter() - t_wall0) < 0.015:
            self.cached_goal = (goal_x, goal_y)
            self.cached_wp = self.world.find_next_waypoint(mx, my, goal_x, goal_y)
            self.last_path_t = t

        wx, wy = self.cached_wp if self.cached_wp is not None else (goal_x, goal_y)
        move_x, move_y = wx - mx, wy - my
        mlen = math.hypot(move_x, move_y)
        if mlen > 1.5:
            move_x /= mlen
            move_y /= mlen
        elif mode in ("CUBE", "LOOT"):
            move_x /= 1.5
            move_y /= 1.5
        else:
            move_x, move_y = 0.0, 0.0

        dx_d, dy_d, dodge_boost = self._dodge_vector(obs)
        if dx_d != 0.0 or dy_d != 0.0:
            move_x += dx_d
            move_y += dy_d
            if dodge_boost and energy > 25.0:
                want_boost = True

        want_jump = False
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

        if energy < 18.0:
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

        # --- Aiming & Weapon Firing (Proper Aim Priority Arbitration!) ------
        fire = [False, False]
        want_reload = False
        aim_yaw = me["aim"][0]
        aim_pitch = 0.0

        # Farm NPCs/crates ONLY when healthy & no enemy mech threats
        farm_target: Optional[Tuple[List[float], List[float]]] = None
        if not recovering and not zone_danger and target_mech is None and hp_frac > 0.78 and len(self.threats) == 0:
            near_npc = min(vis_npcs, key=lambda n: horiz_dist(me["pos"], n["pos"]), default=None)
            near_crate = min(obs["crates"], key=lambda c: horiz_dist(me["pos"], c["pos"]), default=None)
            if near_npc and horiz_dist(me["pos"], near_npc["pos"]) < 60.0:
                np_p = near_npc["pos"]
                if self.world.has_los(my_eye, [np_p[0], np_p[1], np_p[2] + 1.75]):
                    farm_target = ([np_p[0], np_p[1], np_p[2] + 1.75], list(near_npc["vel"]))
            elif near_crate and horiz_dist(me["pos"], near_crate["pos"]) < 50.0:
                cp = near_crate["pos"]
                if self.world.has_los(my_eye, [cp[0], cp[1], cp[2] + 1.0]):
                    farm_target = ([cp[0], cp[1], cp[2] + 1.0], [0.0, 0.0, 0.0])

        if target_mech is not None:
            t_pos = list(target_mech["pos"])
            t_vel = list(target_mech["vel"])
            t_scale = 1.0 + self.size_per_cube * float(target_mech.get("cubes", 0.0))
            t_com = [t_pos[0], t_pos[1], t_pos[2] + 4.5 * t_scale]
            t_hp = float(target_mech.get("hp", 1000.0))
            d_to_target = dist3(my_muzzle, t_com)

            direct_yaw = math.atan2(t_com[1] - my_muzzle[1], t_com[0] - my_muzzle[0])
            direct_pitch = math.atan2(t_com[2] - my_muzzle[2], max(0.1, horiz_dist(my_muzzle, t_com)))
            aim_yaw, aim_pitch = direct_yaw, direct_pitch

            if rear_bearing is not None and not has_los_target:
                aim_yaw, aim_pitch = rear_bearing, 0.0

            # Always shoot back when we have LOS (only hold fire if recovering OUT OF LOS)
            allow_fire = has_los_target or (not recovering and not outnumbered)

            if allow_fire:
                # Choose which weapon controls the torso aim this tick:
                # 1. Charging or ready laser (hitscan)
                # 2. Ready mortar (when >=32m or target behind cover or gun reloading)
                # 3. Gun (lead aim) / Lightning
                aim_owner: Optional[int] = None
                gun_reloading = any(w and w["kind"] == "gun" and float(w.get("reload", 0.0)) > 0.0 for w in me["weapons"])
                mortar_sol: Optional[Tuple[float, float]] = None

                for idx, w in enumerate(me["weapons"]):
                    if w and w["kind"] == "laser":
                        l_range = float(w["stats"].get("range", 280.0))
                        if (w.get("charge", 0.0) > 0.0 or (w["cooldown"] <= 0.05 and has_los_target)) and d_to_target <= l_range:
                            aim_owner = idx
                            break

                if aim_owner is None:
                    for idx, w in enumerate(me["weapons"]):
                        if w and w["kind"] == "mortar" and w["cooldown"] <= 0.08:
                            if (not has_los_target or gun_reloading or d_to_target >= 34.0) and (time.perf_counter() - t_wall0) < 0.020:
                                mortar_sol = solve_mortar_aim(
                                    my_mortar_muzzle, t_pos, t_vel,
                                    float(w["stats"].get("speed", 72.0)),
                                    float(w["stats"].get("gravity", 22.0)),
                                    self.world,
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
                        if w and w["kind"] == "gun" and float(w.get("reload", 0.0)) <= 0.0 and w.get("ammo", 0) > 0 and has_los_target:
                            aim_yaw, aim_pitch = solve_lead_aim(my_muzzle, t_com, t_vel, float(w["stats"].get("speed", 252.0)))
                            break

                # Now evaluate trigger for each slot independently
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
                        tol = max(0.04, math.atan2(2.2 * t_scale, max(1.0, d_to_target)))
                        if charge > 0.0:
                            frac = clamp(charge / max(0.01, full_c), 0.0, 1.0)
                            est_dmg = dmg_full * (min_frac + (1.0 - min_frac) * frac)
                            ready_rel = charge >= full_c - 0.05 or (charge >= min_c and est_dmg >= t_hp * 1.02)
                            if ready_rel and has_los_target and err_y < tol and err_p < tol:
                                fire[slot_i] = False
                            else:
                                fire[slot_i] = True
                        elif cd <= 0.05 and d_to_target <= l_range and has_los_target and err_y < 0.35:
                            fire[slot_i] = True

                    elif kind == "mortar":
                        if mortar_sol is not None and cd <= 0.08:
                            m_yaw, m_pitch = mortar_sol
                            if angle_diff(me["aim"][0], m_yaw) < 0.10 and abs(me["aim"][1] - m_pitch) < 0.10:
                                fire[slot_i] = True

                    elif kind == "lightning":
                        lt_range = float(st.get("range", 45.0))
                        cone_rad = math.radians(float(st.get("cone_deg", 30.0)) * 0.5)
                        if cd <= 0.05 and d_to_target <= lt_range and has_los_target and energy > 22.0:
                            if angle_diff(me["aim"][0], direct_yaw) <= cone_rad * 0.85:
                                fire[slot_i] = True

                    elif kind == "gun":
                        g_range = float(st.get("range", 230.0))
                        g_speed = float(st.get("speed", 252.0))
                        ammo = float(w.get("ammo", 0.0))
                        reloading = float(w.get("reload", 0.0)) > 0.0
                        if ammo <= 3 and not has_los_target and not reloading:
                            want_reload = True
                        if not reloading and ammo > 0 and d_to_target <= g_range and has_los_target:
                            g_yaw, g_pitch = solve_lead_aim(my_muzzle, t_com, t_vel, g_speed)
                            err_y = angle_diff(me["aim"][0], g_yaw)
                            err_p = abs(me["aim"][1] - g_pitch)
                            tol = max(0.065, math.atan2(2.8 * t_scale, max(1.0, d_to_target)))
                            if cd <= 0.05 and err_y < tol and err_p < tol:
                                fire[slot_i] = True

        elif farm_target is not None:
            ft_pos, ft_vel = farm_target
            d_ft = dist3(my_muzzle, ft_pos)
            for slot_i, w in enumerate(me["weapons"]):
                if w and w["kind"] == "gun" and float(w.get("reload", 0.0)) <= 0.0 and w.get("ammo", 0) > 0 and d_ft <= float(w["stats"].get("range", 230.0)):
                    aim_yaw, aim_pitch = solve_lead_aim(my_muzzle, ft_pos, ft_vel, float(w["stats"].get("speed", 252.0)))
                    if w["cooldown"] <= 0.05 and angle_diff(me["aim"][0], aim_yaw) < 0.08:
                        fire[slot_i] = True
                    break

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
                    aim_yaw = wrap_angle(base_yaw + 1.1 * math.sin(t * 1.5))

            for w in me["weapons"]:
                if w and w["kind"] == "gun" and w.get("reload", 0.0) <= 0.0:
                    if float(w.get("ammo", 0.0)) < float(w["stats"].get("mag", 12.0)) * 0.5:
                        want_reload = True

        act["aim"] = [round(wrap_angle(aim_yaw), 4), round(clamp(aim_pitch, self.min_pitch, self.max_pitch), 4)]
        act["fire"] = fire
        if want_reload:
            act["reload"] = True

        mf.draw.label(f"{mode} | hp {hp:.0f} | c {me['cubes']:.0f}")
        return act


if __name__ == "__main__":
    mf.run(Bot)