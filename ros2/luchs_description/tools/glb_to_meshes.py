#!/usr/bin/env python3
"""Regenerate the Luchs A 4WS visual meshes from the CAD GLB export.

Usage:
    python3 glb_to_meshes.py <Luchs_A-4WS_IP54.glb> <output meshes dir>

Requires: numpy, trimesh, fast-simplification.

CAD convention of the GLB (metres): X = width (CAD +X = vehicle LEFT),
Y = up, Z = length (CAD +Z = vehicle FRONT).
Output convention (REP-103, metres):  x forward, y left, z up.
    x_ros = Z_cad,  y_ros = X_cad,  z_ros = Y_cad - AXLE_Y_CAD
so every mesh origin sits at wheel-axle height (base_link height).

Front/rear decision: the CAD +Z end carries a separate fascia bracket
with a centred, forward-facing sensor housing (GLB parts 002/006/075);
the CAD -Z end carries a hanging plate with two electrode strips
(auto-docking charging contacts, parts 076-086), which is placed at
the rear. Re-check this against the real vehicle (see README).
"""
import sys
import numpy as np
import trimesh
import fast_simplification as fs

AXLE_Y_CAD = -0.2453      # mean wheel-centre height in CAD (from tyre bounding boxes)
WHEEL_X_CAD = 0.2501      # half track, CAD X
WHEEL_Z_CAD = 0.2972      # half wheelbase, CAD Z

# GLB node numbers ("Part-NNN") grouped per corner, named in ROS terms.
WHEEL = {
    'front_left':  [105, 106, 109, 110],
    'front_right': [54, 57, 58, 59],
    'rear_left':   [26, 29, 30, 31],
    'rear_right':  [113, 114, 117, 118],
}
STEER = {
    'front_left':  [48, 49, 50, 51, 52, 107, 108, 111, 112],
    'front_right': [53, 55, 56, 60, 61, 62, 63, 64, 65],
    'rear_left':   [25, 27, 28, 32, 33, 34, 35, 36, 37],
    'rear_right':  [19, 20, 21, 22, 23, 115, 116, 119, 120],
}
CORNER_SIGN = {'front_left': (1, 1), 'front_right': (1, -1),
               'rear_left': (-1, 1), 'rear_right': (-1, -1)}

R = np.array([[0, 0, 1],    # x_ros <- Z_cad
              [1, 0, 0],    # y_ros <- X_cad
              [0, 1, 0]])   # z_ros <- Y_cad


def load_parts(path):
    scene = trimesh.load(path)
    parts = {}
    for node in scene.graph.nodes_geometry:
        T, gname = scene.graph[node]
        m = scene.geometry[gname].copy()
        m.apply_transform(T)
        v = (R @ m.vertices.T).T
        v[:, 2] -= AXLE_Y_CAD
        parts[int(node.split('-')[1])] = trimesh.Trimesh(v, m.faces, process=True)
    return parts


def build(parts, ids, budget, offset=(0.0, 0.0, 0.0)):
    """Decimate each CAD part on its own (shared ratio, so the triangle budget
    is spread evenly), clamp the result to the part's original bounding box
    (quadric decimation can push a few vertices of thin parts outwards), then
    merge and move into the link frame."""
    total = sum(len(parts[i].faces) for i in ids)
    ratio = min(1.0, budget / total)
    out = []
    for i in ids:
        m = parts[i]
        target = max(min(len(m.faces), 24), int(len(m.faces) * ratio))
        if target < len(m.faces):
            v, f = fs.simplify(m.vertices.astype(np.float32), m.faces.astype(np.int32),
                               target_reduction=1.0 - target / len(m.faces))
            v = np.clip(v.astype(np.float64), m.bounds[0], m.bounds[1])
            m = trimesh.Trimesh(v, f, process=True)
        out.append(m)
    m = trimesh.util.concatenate(out)
    m.apply_translation(-np.asarray(offset))
    m.remove_unreferenced_vertices()
    return m


def corner_origin(c):
    sx, sy = CORNER_SIGN[c]
    return (sx * WHEEL_Z_CAD, sy * WHEEL_X_CAD, 0.0)


def main(glb, outdir):
    parts = load_parts(glb)
    module_ids = {i for d in (WHEEL, STEER) for ids in d.values() for i in ids}
    body_ids = sorted(set(parts) - module_ids)

    body = build(parts, body_ids, 25000)
    wheel = build(parts, WHEEL['front_left'], 6500, corner_origin('front_left'))
    # FL and RR modules are point-symmetric (rotation by pi about z), as are FR and RL,
    # so two mesh variants cover all four corners without negative scaling.
    steer = build(parts, STEER['front_left'], 6500, corner_origin('front_left'))
    steer_m = build(parts, STEER['front_right'], 6500, corner_origin('front_right'))

    for name, m in [('luchs_a_4ws_body', body), ('luchs_a_wheel', wheel),
                    ('luchs_a_4ws_steer_module', steer),
                    ('luchs_a_4ws_steer_module_mirror', steer_m)]:
        m.export(f'{outdir}/{name}.stl', file_type='stl')  # trimesh writes binary STL
        b = m.bounds
        print(f'{name:34s} tris={len(m.faces):6d} min={np.round(b[0], 4)} max={np.round(b[1], 4)}')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
