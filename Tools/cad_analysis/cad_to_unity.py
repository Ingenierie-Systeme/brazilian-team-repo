#!/usr/bin/env python3
"""Converts the CAD of the robot fish (Onshape URDF + STL) into meshes for Unity.

What it does:
  1. Places every part in the robot frame (forward kinematics of the URDF, as cad_analysis.py).
  2. Converts the coordinates to the Unity frame of the robot (forward = −X, up = +Y, right = +Z). The CAD frame
     is right-handed and Unity is left-handed. Unity's OBJ importer already handles this: it negates X (and
     reverses the triangle order) on import. The OBJ files are therefore written in the right-handed frame
     (−X, Y, Z) of the final Unity frame, which is a pure rotation of the CAD frame (no mirror, same triangle order).
  3. Moves the parts of the two mechanisms to their reference pose and expresses them relative to their pivot:
       - sliding mass: carriage (relative to its rod pin), servo horn (relative to the servo shaft) and rod
         (relative to the carriage pin);
       - tail: servo hub, arm and fin, turned back to 0° (the CAD was saved with the fin deflected) and
         expressed relative to the servo axis.
  4. Simplifies the heavy meshes (quadric decimation; the Raspberry Pi board alone has 1.46 M triangles).
  5. Writes one OBJ file per part and a manifest (JSON) with the hierarchy, pivots, colors and the
     mechanism geometry. Unity reads them with Assets/AUV/Editor/RobotFishPrefabBuilder.cs.

Usage (needs the virtual environment of README.md for the mesh simplification):
  Tools/cad_analysis/.venv/bin/python Tools/cad_analysis/cad_to_unity.py <CAD folder> [--out Assets/AUV/Models/RobotFish]
"""
import argparse
import json
import os

import fast_simplification
import numpy as np

import cad_analysis as ca

HERE = os.path.dirname(os.path.abspath(__file__))

# Maximum number of triangles kept per part (parts with fewer triangles are not simplified)
DEFAULT_BUDGET = 12000
BUDGET = {'rpi_navigator': 30000, '_hs_5496mh': 12000, 'underwater_servo': 24000, 'rail_400': 26000, 'rail_400_1': 26000,
          'dome_cercle': 40000, 'cap_rear': 16000, 'cap_front': 16000, 'tray_rpi': 18000}

# Parts by function: (group in the Unity hierarchy, moving element or None, material)
PARTS = {
    'tube': ('Hull', None, 'acrylic'),
    'cap_front': ('Hull', None, 'cad'),
    'cap_rear': ('Hull', None, 'cad'),
    'dome_cercle': ('Hull', None, 'acrylic'),
    'camera': ('Camera', None, 'cad'),
    'camera_mount': ('Camera', None, 'cad'),
    'rail_400': ('Electronics', None, 'cad'),
    'rail_400_1': ('Electronics', None, 'cad'),
    'tray_rpi': ('Electronics', None, 'cad'),
    'rpi_navigator': ('Electronics', None, 'cad'),
    'tray_battery': ('Electronics', None, 'cad'),
    'battery': ('Electronics', None, 'cad'),
    '_hs_5496mh': ('PitchMechanism', None, 'cad'),
    'conector_servo': ('PitchMechanism', None, 'cad'),
    'conector_servo_1': ('PitchMechanism', 'horn', 'cad'),
    'haste': ('PitchMechanism', 'rod', 'cad'),
    'conector_pesos': ('PitchMechanism', 'carriage', 'cad'),
    'clamp': ('TailDrive', None, 'cad'),
    'servo_mount': ('TailDrive', None, 'cad'),
    'underwater_servo': ('TailDrive', None, 'cad'),
    'underwater_servo_hub': ('TailDrive', 'tail', 'cad'),
    'conector_calda': ('TailDrive', 'tail', 'cad'),
    'calda': ('TailDrive', 'tail', 'fin'),
    'calda_lateral': ('DorsalFin', None, 'fin'),
}


def to_unity(p):
    """CAD points (…, 3) → Unity robot frame."""
    p = np.asarray(p)
    return np.stack([p[..., 1], p[..., 2], -p[..., 0]], axis=-1)


def joint_frames(cad):
    """World position and axis of every URDF joint (CAD frame)."""
    import xml.etree.ElementTree as ET
    root = ET.parse(os.path.join(cad, 'urdf', 'assembly.urdf')).getroot()
    children = {}
    for j in root.findall('joint'):
        children.setdefault(j.find('parent').get('link'), []).append(j)
    world, joints, stack = {'root': np.eye(4)}, {}, ['root']
    while stack:
        parent = stack.pop()
        for j in children.get(parent, []):
            t = world[parent] @ ca.origin_tf(j.find('origin'))
            world[j.find('child').get('link')] = t
            axis = np.array([float(v) for v in j.find('axis').get('xyz').split()]) if j.find('axis') is not None else np.zeros(3)
            joints[j.get('name')] = (t[:3, 3], t[:3, :3] @ axis)
            stack.append(j.find('child').get('link'))
    return joints


def link_colors(cad):
    """RGBA color of each link in the URDF (gray when the link has none)."""
    import xml.etree.ElementTree as ET
    colors = {}
    for link in ET.parse(os.path.join(cad, 'urdf', 'assembly.urdf')).getroot().findall('link'):
        c = link.find('visual/material/color')
        colors[link.get('name')] = [float(v) for v in c.get('rgba').split()] if c is not None else [0.7, 0.7, 0.7, 1.0]
    return colors


def weld(tri):
    """Triangle soup (n, 3, 3) → shared vertices + faces (STL files repeat every vertex)."""
    pts = tri.reshape(-1, 3)
    key = np.round(pts / 1e-7).astype(np.int64)
    _, first, inverse = np.unique(key, axis=0, return_index=True, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])]
    return pts[first], faces


def cluster(verts, faces, cell):
    """Vertex clustering: every vertex moves to the average of its grid cell (size `cell`); faces that collapse
    are removed. Used when quadric decimation cannot go further (many tiny separate solids, e.g. PCB components)."""
    key = np.floor(verts / cell).astype(np.int64)
    _, inverse = np.unique(key, axis=0, return_inverse=True)
    inverse = inverse.ravel()
    n = inverse.max() + 1
    merged = np.zeros((n, 3))
    np.add.at(merged, inverse, verts)
    merged /= np.bincount(inverse, minlength=n)[:, None]
    f = inverse[faces]
    f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]   # drop collapsed faces
    _, keep = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
    f = f[np.sort(keep)]
    used, f = np.unique(f, return_inverse=True)
    return merged[used], f.reshape(-1, 3)


def rotate_z(points, center, angle):
    """Rotates CAD points about a vertical axis through center (counterclockwise seen from above)."""
    c, s = np.cos(angle), np.sin(angle)
    d = points - center
    return np.stack([center[0] + c * d[..., 0] - s * d[..., 1], center[1] + s * d[..., 0] + c * d[..., 1], points[..., 2]], axis=-1)


def write_obj(path, verts, faces, name):
    """Writes Unity-frame vertices as an OBJ file: x is negated because Unity negates it again on import."""
    with open(path, 'w') as f:
        f.write(f'# {name}: robot fish part converted from the CAD by Tools/cad_analysis/cad_to_unity.py (meters)\n')
        f.write(f'o {name}\n')
        f.write(''.join(f'v {-v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n' for v in verts))
        f.write(''.join(f'f {a + 1} {b + 1} {c + 1}\n' for a, b, c in faces))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cad', help='CAD folder (with urdf/assembly.urdf and meshes/)')
    ap.add_argument('--out', default=os.path.join(HERE, '..', '..', 'Assets', 'AUV', 'Models', 'RobotFish'))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    parts = ca.load_parts(args.cad)
    joints = joint_frames(args.cad)
    colors = link_colors(args.cad)

    # ---- mechanism geometry (CAD frame, from the URDF joints)
    shaft = joints['cylindrical_7_loop_closure'][0]               # HS-5496MH output shaft (lateral axis)
    horn_pin = joints['planar_10'][0]                             # horn ↔ rod pin
    carriage_pin = joints['cylindrical_10'][0]                    # rod ↔ carriage pin (carriage at its front stop in the CAD)
    tail_axis = joints['cylindrical_1'][0]                        # D955TW output axis (vertical)
    fin = parts['calda']['tri'].reshape(-1, 3)
    chord = fin.mean(0)[:2] - tail_axis[:2]
    fin_deflection = np.arctan2(chord[0], chord[1])               # CAD fin angle (+ toward CAD +x = robot left)

    manifest = {'parts': [], 'mechanisms': {}}
    total_in = total_out = 0
    for name, (group, moving, material) in PARTS.items():
        tri = parts[name]['tri'].copy()
        pivot_cad = np.zeros(3)
        if moving == 'tail':                                      # turn back to 0°, express relative to the servo axis
            tri = rotate_z(tri, tail_axis, fin_deflection)
            pivot_cad = np.array([tail_axis[0], tail_axis[1], parts['underwater_servo_hub']['cen'][2]])
        elif moving == 'horn':
            pivot_cad = shaft
        elif moving in ('rod', 'carriage'):
            pivot_cad = carriage_pin
        verts, faces = weld(tri - pivot_cad)
        verts = to_unity(verts)                                   # mirrored frame (the OBJ writer and Unity mirror it back)
        n_in = len(faces)
        budget = BUDGET.get(name, DEFAULT_BUDGET)
        if n_in > budget:                                         # 1. quadric decimation (keeps the shape)
            verts, faces = fast_simplification.simplify(verts.astype(np.float32), faces.astype(np.int64), target_reduction=1 - budget / n_in)
        cell = 0.0005
        while len(faces) > budget * 1.5:                          # 2. vertex clustering when decimation is stuck
            verts, faces = cluster(verts.astype(float), faces, cell)
            cell *= 1.4
        total_in, total_out = total_in + n_in, total_out + len(faces)
        write_obj(os.path.join(args.out, f'{name}.obj'), verts, faces, name)
        manifest['parts'].append({'name': name, 'file': f'{name}.obj', 'group': group, 'moving': moving or '',
                                  'pivot': to_unity(pivot_cad).round(6).tolist(), 'material': material,
                                  'color': colors.get(name, [0.7, 0.7, 0.7, 1.0]), 'triangles': int(len(faces))})
        print(f'{name:22s} {n_in:8d} → {len(faces):6d} triangles')

    manifest['mechanisms'] = {
        'shaft': to_unity(shaft).round(6).tolist(),
        'hornPinAtFrontStop': to_unity(horn_pin).round(6).tolist(),
        'carriagePinAtFrontStop': to_unity(carriage_pin).round(6).tolist(),
        'hornRadius': round(float(np.linalg.norm((horn_pin - shaft)[1:])), 6),
        'rodLength': round(float(np.linalg.norm((horn_pin - carriage_pin)[1:])), 6),
        'tailAxis': to_unity(np.array([tail_axis[0], tail_axis[1], parts['underwater_servo_hub']['cen'][2]])).round(6).tolist(),
        'cadFinDeflectionDeg': round(float(np.degrees(fin_deflection)), 2),
    }
    with open(os.path.join(args.out, 'robot_fish_parts.json'), 'w') as f:
        json.dump(manifest, f, indent=2)
    print(f'\ntotal {total_in} → {total_out} triangles; mechanisms: {json.dumps(manifest["mechanisms"])}')


if __name__ == '__main__':
    main()
