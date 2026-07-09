#!/usr/bin/env python3
"""Generate a centerline waypoint CSV for the levine_blocked sim map.

The blocked Levine ring is a rectangle of corridor centerlines (measured from
the map pixels: corridors are ~1.6-1.7 m wide, resolution 0.05 m/px, origin
-51.225). Corners are rounded with a fixed radius and the whole loop is
sampled at a constant spacing. Speeds are assigned by curvature: straights get
v_max, corner arcs get v_corner.

Run from anywhere:  python3 generate_centerline_waypoints.py [out.csv]
"""
import math
import sys

# Corridor centerlines in map/world coordinates (see gap_follow verification):
#   left  x = -13.70   right x =  9.70
#   top   y =   8.62   bottom y = -0.17
X_L, X_R = -13.70, 9.70
Y_B, Y_T = -0.17, 8.62
R = 1.0          # corner radius (m); corridor half-width is ~0.8
SPACING = 0.10   # waypoint spacing (m)
V_MAX = 3.5
V_CORNER = 1.8


def build_path():
    """Counter-clockwise loop starting near the sim spawn (0, -0.17) heading +x."""
    segs = []
    # straight segments (shortened by R at each end), then 90-deg arcs
    straights = [
        ((0.0, Y_B), (X_R - R, Y_B)),          # bottom, eastbound (from spawn)
        ((X_R, Y_B + R), (X_R, Y_T - R)),      # right, northbound
        ((X_R - R, Y_T), (X_L + R, Y_T)),      # top, westbound
        ((X_L, Y_T - R), (X_L, Y_B + R)),      # left, southbound
        ((X_L + R, Y_B), (0.0, Y_B)),          # bottom, eastbound back to spawn
    ]
    arcs = [  # (center, start angle) each sweeping +90 deg CCW
        ((X_R - R, Y_B + R), -math.pi / 2),    # bottom-right
        ((X_R - R, Y_T - R), 0.0),             # top-right
        ((X_L + R, Y_T - R), math.pi / 2),     # top-left
        ((X_L + R, Y_B + R), math.pi),         # bottom-left
    ]
    for i in range(4):
        segs.append(('line', straights[i]))
        segs.append(('arc', arcs[i]))
    segs.append(('line', straights[4]))

    pts = []
    for kind, geo in segs:
        if kind == 'line':
            (x0, y0), (x1, y1) = geo
            d = math.hypot(x1 - x0, y1 - y0)
            n = max(1, int(d / SPACING))
            for i in range(n):
                t = i / n
                pts.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0), V_MAX))
        else:
            (cx, cy), a0 = geo
            arc_len = R * math.pi / 2
            n = max(1, int(arc_len / SPACING))
            for i in range(n):
                a = a0 + (i / n) * (math.pi / 2)
                pts.append((cx + R * math.cos(a), cy + R * math.sin(a), V_CORNER))
    return pts


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'levine_blocked_centerline.csv'
    pts = build_path()
    with open(out, 'w') as f:
        f.write('x,y,yaw,speed\n')
        for i, (x, y, v) in enumerate(pts):
            nx, ny, _ = pts[(i + 1) % len(pts)]
            yaw = math.atan2(ny - y, nx - x)
            f.write(f'{x:.4f},{y:.4f},{yaw:.4f},{v:.2f}\n')
    print(f'{len(pts)} waypoints -> {out}')


if __name__ == '__main__':
    main()
