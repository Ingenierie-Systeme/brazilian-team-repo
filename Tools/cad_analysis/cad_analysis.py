#!/usr/bin/env python3
"""Physical properties of the robot fish estimated from its CAD model (Onshape URDF + STL meshes).

What it does:
  1. Reads the URDF, places every part in the robot frame (forward kinematics, all joints at 0).
  2. Measures each part from its mesh: volume, centroid, inertia (exact integration over the closed mesh).
  3. Gives each part a mass from assumptions.yaml (CAD volume × density, or a datasheet mass).
  4. Computes the displaced volume, centers of mass and buoyancy, buoyancy for several moving masses, pitch authority of the
     sliding mass, inertia, and the geometry of the tail and dorsal fins.
  5. Writes a Markdown report and two images into the Docs folder.

The Onshape export has no materials: the URDF "mass" of each part is its volume in m³ (density 1),
so the real masses must come from assumptions.yaml (or, better, from weighing the parts).

Usage:
  python3 cad_analysis.py <CAD folder containing urdf/ and meshes/> [--docs <Docs folder>]

Frames:
  CAD frame:   Y = long axis (front/dome = −Y, tail = +Y), Z = up, X = lateral.
  Unity frame (local axes of the RobotFish prefab): forward = −X, up = +Y, right = +Z, so
               Unity = (CAD y, CAD z, −CAD x).
"""
import argparse
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
RHO_WATER = 1000.0
G = 9.81


# ---------------------------------------------------------------- geometry helpers
def rpy_matrix(r, p, y):
    """Rotation matrix of URDF roll-pitch-yaw angles (applied as Rz·Ry·Rx)."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def origin_tf(el):
    """4×4 transform of a URDF <origin> element."""
    t = np.eye(4)
    if el is not None:
        t[:3, :3] = rpy_matrix(*[float(v) for v in el.get('rpy', '0 0 0').split()])
        t[:3, 3] = [float(v) for v in el.get('xyz', '0 0 0').split()]
    return t


def read_stl(path):
    """Triangles of a binary STL file, shape (n, 3, 3), in meters."""
    with open(path, 'rb') as f:
        data = f.read()
    n = struct.unpack('<I', data[80:84])[0]
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return rec['v'].reshape(-1, 3, 3).astype(float)


def mesh_properties(tri):
    """Volume, centroid and inertia tensor about the centroid (unit density) of a closed mesh.

    Each triangle forms a tetrahedron with the origin; volumes and second moments are summed
    (signed, so the result is exact for a closed, consistently oriented mesh).
    """
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    v6 = np.einsum('ij,ij->i', a, np.cross(b, c))          # 6 × signed tetra volume
    vol = v6.sum() / 6
    centroid = (v6[:, None] * (a + b + c)).sum(0) / (24 * vol)
    # second moments ∫ x_i x_j dV of each tetra (origin, a, b, c)
    s = a + b + c
    cov = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            cov[i, j] = (v6 / 120 * (a[:, i] * a[:, j] + b[:, i] * b[:, j] + c[:, i] * c[:, j] + s[:, i] * s[:, j])).sum()
    cov -= vol * np.outer(centroid, centroid)                # move to the centroid
    inertia = np.trace(cov) * np.eye(3) - cov                # inertia tensor (unit density)
    return vol, centroid, inertia


def to_unity(v):
    """CAD vector → Unity local vector (forward −X, up +Y, right +Z)."""
    return np.array([v[1], v[2], -v[0]])


def to_unity_tensor(t):
    """CAD inertia tensor → Unity local axes."""
    p = np.array([[0, 1, 0], [0, 0, 1], [-1, 0, 0]])
    return p @ t @ p.T


# ---------------------------------------------------------------- load the CAD
def load_parts(cad):
    root = ET.parse(os.path.join(cad, 'urdf', 'assembly.urdf')).getroot()
    links = {l.get('name'): l for l in root.findall('link')}
    children = {}
    for j in root.findall('joint'):
        children.setdefault(j.find('parent').get('link'), []).append(j)
    world, stack = {'root': np.eye(4)}, ['root']                  # forward kinematics from the root link
    while stack:
        parent = stack.pop()
        for j in children.get(parent, []):
            child = j.find('child').get('link')
            world[child] = world[parent] @ origin_tf(j.find('origin'))
            stack.append(child)
    parts = {}
    for name, link in links.items():
        vis = link.find('visual')
        if vis is None or name not in world:
            continue
        mesh = vis.find('geometry/mesh').get('filename').split('/')[-1]
        t = world[name] @ origin_tf(vis.find('origin'))
        tri = read_stl(os.path.join(cad, 'meshes', mesh)) @ t[:3, :3].T + t[:3, 3]
        vol, cen, inertia = mesh_properties(tri)
        parts[name] = dict(tri=tri, vol=vol, cen=cen, inertia=inertia, lo=tri.reshape(-1, 3).min(0), hi=tri.reshape(-1, 3).max(0))
    return parts


def fin_geometry(tri):
    """Planform area, chord, span and thickness of a flat fin (plate) mesh."""
    pts = tri.reshape(-1, 3)
    c = pts.mean(0)
    _, _, vt = np.linalg.svd(pts - c)
    normal = vt[2]
    cross = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    area = 0.5 * np.linalg.norm(cross, axis=1)
    on_face = np.abs(cross @ normal) / np.maximum(np.linalg.norm(cross, axis=1), 1e-12) > 0.95
    return area[on_face].sum() / 2, normal, (pts @ normal).ptp()


# ---------------------------------------------------------------- analysis
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cad', help='CAD folder (with urdf/assembly.urdf and meshes/)')
    ap.add_argument('--docs', default=os.path.join(HERE, '..', '..', 'Docs'), help='Docs folder for the report and images')
    ap.add_argument('--assumptions', default=os.path.join(HERE, 'assumptions.yaml'))
    args = ap.parse_args()
    A = yaml.safe_load(open(args.assumptions))
    parts = load_parts(args.cad)

    # ---- 1. mass of each part (assumptions.yaml) and mass budget
    rows = []                                                   # (name, material, status, volume cm³, mass kg, centroid, inertia about centroid)
    for name, spec in A['parts'].items():
        p = parts[name]
        vol_cm3 = p['vol'] * 1e6
        if 'mass' in spec:
            mass = spec['mass'] / 1000
        elif 'servo_mass' in spec:
            mass = (spec['servo_mass'] + (vol_cm3 - spec['servo_volume_cm3']) * spec['housing_density']) / 1000
        else:
            mass = vol_cm3 * spec['density'] / 1000
        rows.append((name, spec['material'], spec['status'], vol_cm3, mass, p['cen'], p['inertia'] * mass / p['vol']))
    for name, spec in A['extra_masses'].items():
        at = parts[spec['at_part']]['cen'] if 'at_part' in spec else np.array(spec['at'], float)
        rows.append((name, 'not in the CAD', spec['status'], 0.0, spec['mass'] / 1000, at, np.zeros((3, 3))))
    target = A.get('structure_mass_target')                     # optional: total structure mass given by the team
    if target:
        missing = target['mass_g'] / 1000 - sum(r[4] for r in rows)
        rows.append(('unaccounted', 'difference with the team estimate of the robot mass', target['status'], 0.0,
                     missing, np.array(target['at'], float), np.zeros((3, 3))))
    m_dry = sum(r[4] for r in rows)
    com_dry = sum(r[4] * r[5] for r in rows) / m_dry

    # ---- 2. displaced volume: hull envelope (tube + cap discs + dome) + every part outside the hull
    r_tube = parts['tube']['hi'][0]                              # 55 mm
    tube_len = parts['tube']['hi'][1] - parts['tube']['lo'][1]   # 400 mm
    r_cap = parts['cap_front']['hi'][0]                          # 57.7 mm
    cap_out = parts['cap_rear']['hi'][1] - tube_len / 2          # 8 mm outside the tube on each end
    r_dome = parts['dome_cercle']['hi'][0]
    dome_base = parts['cap_front']['lo'][1]                      # dome starts at the front face of the cap
    hull = [
        ('tube envelope', np.pi * r_tube**2 * tube_len, np.zeros(3)),
        ('cap discs', 2 * np.pi * r_cap**2 * cap_out, np.zeros(3)),
        ('dome', 2 / 3 * np.pi * r_dome**3, np.array([0, dome_base - 3 / 8 * r_dome, 0])),
    ]
    outside = ['clamp', 'servo_mount', 'underwater_servo', 'underwater_servo_hub', 'conector_calda', 'calda', 'calda_lateral']
    hull += [(n, parts[n]['vol'], parts[n]['cen']) for n in outside]
    vol_disp = sum(v for _, v, _ in hull)
    cob = sum(v * c for _, v, c in hull) / vol_disp

    # ---- 3. moving mass on the tube axis (pitch control; its weight is adjustable, the robot carries no ballast)
    mm_spec = A['moving_mass']
    m_neutral = RHO_WATER * vol_disp - m_dry                      # moving mass that makes the robot neutral
    m_move = round(m_neutral, 3) if mm_spec['mass_g'] == 'neutral' else mm_spec['mass_g'] / 1000
    move_pos = np.array(mm_spec['center_mm'], float) / 1000       # center of its travel
    half_travel = mm_spec['travel_mm'] / 2000                     # ± around the center
    m_total = m_dry + m_move
    com = (m_dry * com_dry + m_move * move_pos) / m_total         # moving mass at the center of its travel
    bg = cob[2] - com[2]                                          # CoM below CoB
    buoyancy_pct = (RHO_WATER * vol_disp / m_total - 1) * 100     # + = floats
    # position of the moving mass along the axis that puts the CoM right under the CoB (robot level), + = forward
    level_pos = -(m_total * cob[1] - m_dry * com_dry[1]) / m_move - move_pos[1]
    rest_pitch = np.degrees(np.arctan2(com[1] - cob[1], bg))      # + = CoM behind the CoB → nose up at rest

    def balance(m):
        """Total mass, BG, buoyancy and maximum static pitch with a moving mass m [kg] (at the center, then at an end)."""
        mt = m_dry + m
        c = (m_dry * com_dry + m * move_pos) / mt
        b = cob[2] - c[2]
        pitch = np.degrees(np.arctan2(m * half_travel, mt * b))  # tan θ = m·d / (M·BG)
        return mt, b, (RHO_WATER * vol_disp / mt - 1) * 100, pitch

    # ---- 4. inertia of the structure about its own CoM (the simulation adds the moving mass at its current position)
    inertia = np.zeros((3, 3))
    for r in rows:
        d = r[5] - com_dry
        inertia += r[6] + r[4] * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    added_rot = RHO_WATER * np.pi * r_tube**2 * tube_len**3 / 12  # slender cylinder, about a lateral axis
    added_lat = RHO_WATER * np.pi * r_tube**2 * tube_len          # added mass sideways / vertical

    # ---- 6. fins
    tail_area, tail_n, tail_t = fin_geometry(parts['calda']['tri'])
    tail_lo, tail_hi = parts['calda']['lo'], parts['calda']['hi']
    hinge = parts['underwater_servo_hub']['cen']
    chord_dir = np.cross(tail_n, [0, 0, 1])
    chord_dir *= np.sign(chord_dir[1])
    tail_pts = parts['calda']['tri'].reshape(-1, 3)
    along = (tail_pts - hinge) @ chord_dir
    le, te = along.min(), along.max()
    chord, span = te - le, tail_hi[2] - tail_lo[2]
    tail_cp = le + chord / 4                                      # quarter-chord, behind the hinge
    tail_deflection = np.degrees(np.arctan2(chord_dir[0], chord_dir[1]))
    dors_area, _, dors_t = fin_geometry(parts['calda_lateral']['tri'])
    dors = parts['calda_lateral']
    dors_cp = np.array([0, dors['lo'][1] + 0.6 * (dors['hi'][1] - dors['lo'][1]), dors['lo'][2] + 0.4 * (dors['hi'][2] - dors['lo'][2])])
    aspect = span**2 / tail_area
    servo = A['tail_servo']

    # ---- 7. overall dimensions
    allpts = np.vstack([p['tri'].reshape(-1, 3) for p in parts.values()])
    size = allpts.max(0) - allpts.min(0)

    # ---------------------------------------------------------------- report
    mm = lambda v: '(' + ', '.join(f'{x * 1000:.1f}' for x in v) + ')'
    L = []
    L.append('# Robot fish: properties estimated from the CAD\n')
    L.append('*Generated by `Tools/cad_analysis/cad_analysis.py` from the Onshape export and `Tools/cad_analysis/assumptions.yaml`. '
             'Do not edit by hand: change the assumptions and run the script again (see `Tools/cad_analysis/README.md`).*\n')
    L.append('Frames: CAD = (X lateral, Y long axis with the front at −Y, Z up); Unity = local axes of the `RobotFish` prefab '
             '(forward = −X, up = +Y, right = +Z), Unity = (CAD y, CAD z, −CAD x). Positions are relative to the tube center.\n')
    L.append('![Robot fish, side and top views](images/robot_fish_views.png)\n')
    L.append('## Overall dimensions\n')
    L.append(f'| | Value |\n| --- | --- |\n| Total length (dome tip → tail fin tip) | {size[1] * 1000:.0f} mm |\n'
             f'| Hull (dome tip → rear cap) | {(parts["cap_rear"]["hi"][1] - parts["dome_cercle"]["lo"][1]) * 1000:.0f} mm |\n'
             f'| Tube | Ø{r_tube * 2000:.0f} mm outer, {tube_len * 1000:.0f} mm long |\n'
             f'| Width (rear clamp) | {size[0] * 1000:.0f} mm |\n| Height (tail fin bottom → dorsal fin top) | {size[2] * 1000:.0f} mm |\n')
    L.append('## Mass budget\n')
    L.append('| Part | Material / source | Status | CAD volume [cm³] | Mass [g] | Centroid CAD [mm] |\n| --- | --- | --- | --- | --- | --- |')
    for name, mat, st, v, m, c, _ in rows:
        L.append(f'| `{name}` | {mat} | {st} | {v:.1f} | {m * 1000:.0f} | {mm(c)} |')
    L.append(f'| **Total structure** (without the moving mass) | | | | **{m_dry * 1000:.0f}** | {mm(com_dry)} |\n')
    L.append('The robot carries no ballast: its weight is the structure above plus the moving mass.\n')
    L.append('## Buoyancy and balance\n')
    L.append('| Quantity | Value |\n| --- | --- |')
    L.append(f'| Displaced volume | {vol_disp * 1e3:.2f} L (' + ', '.join(f'{n} {v * 1e3:.2f}' for n, v, _ in hull[:3]) + f', parts outside the hull {sum(v for _, v, _ in hull[3:]) * 1e3:.2f}) → the water pushes up with the weight of {vol_disp * 1e3:.2f} kg |')
    L.append(f'| Structure | {m_dry:.2f} kg |')
    L.append(f'| Moving mass (current) | {m_move * 1000:.0f} g, on the tube axis{" (set to the neutral value)" if mm_spec["mass_g"] == "neutral" else ""} |')
    L.append(f'| Total mass | {m_total:.2f} kg → buoyancy **{buoyancy_pct:+.0f} %** of the weight ({"neutral" if abs(buoyancy_pct) < 0.5 else "floats" if buoyancy_pct > 0 else "sinks"}) |')
    L.append(f'| Moving mass for neutral buoyancy | **{m_neutral * 1000:.0f} g** |')
    L.append(f'| Center of buoyancy (CAD) | {mm(cob)} mm |')
    L.append(f'| Center of mass (CAD), moving mass centered | {mm(com)} mm |')
    L.append(f'| CoM below CoB (BG) | {bg * 1000:.1f} mm → righting moment {m_total * G * bg:.2f} N·m/rad |')
    L.append(f'| CoM behind the CoB | {(com[1] - cob[1]) * 1000:.1f} mm → resting pitch {rest_pitch:+.1f}° (+ = nose up) with the moving mass centered |')
    L.append(f'| Moving mass position for a level robot | {level_pos * 1000:.0f} mm forward of the tube center ({"within" if abs(level_pos) <= half_travel else "**outside**"} its ±{half_travel * 1000:.0f} mm travel) |\n')
    L.append('The structure masses are estimates (CAD volume × assumed material), so the buoyancy is uncertain: '
             'weighing the robot is the best way to confirm it.\n')
    L.append('## Moving mass (pitch)\n')
    L.append(f'Travel: {mm_spec["travel_mm"]} mm along the tube axis (±{half_travel * 1000:.0f} mm), speed {mm_spec["speed_mm_s"]} mm/s (estimate). '
             'The maximum static pitch is reached with the mass at one end of its travel (submerged robot): tan θ = m·d / (M·BG).\n')
    L.append('| Moving mass | Total mass | Buoyancy | BG | Maximum static pitch |\n| --- | --- | --- | --- | --- |')
    cases = [(0.1, '100 g'), (0.5, '500 g'), (1.0, '1000 g'), (m_neutral, f'{m_neutral * 1000:.0f} g (neutral)')]
    if abs(m_move - m_neutral) > 0.002:
        cases.insert(0, (m_move, f'{m_move * 1000:.0f} g (current)'))
    for m, label in cases:
        mt, b, bp, pmax = balance(m)
        L.append(f'| {label} | {mt:.2f} kg | {bp:+.0f} % | {b * 1000:.1f} mm | ±{pmax:.0f}° |')
    L.append('')
    L.append('## Inertia\n')
    iu = to_unity_tensor(inertia)
    L.append('Structure only, about its own center of mass (the simulation adds the moving mass at its current position).\n')
    L.append('| Axis (Unity) | Rigid body [kg·m²] | Added (water) [kg·m²] | Total used [kg·m²] |\n| --- | --- | --- | --- |')
    L.append(f'| roll (x, long axis) | {iu[0, 0]:.4f} | 0 | {iu[0, 0]:.4f} |')
    L.append(f'| yaw (y, vertical) | {iu[1, 1]:.4f} | {added_rot:.4f} | {iu[1, 1] + added_rot:.4f} |')
    L.append(f'| pitch (z, lateral) | {iu[2, 2]:.4f} | {added_rot:.4f} | {iu[2, 2] + added_rot:.4f} |\n')
    L.append(f'Hull added mass sideways and vertically: {added_lat:.2f} kg (not used in the simulation, see `RobotFish_Model.md`).\n')
    L.append('## Fins\n')
    L.append('| | Tail fin (`calda`) | Dorsal fin (`calda_lateral`) |\n| --- | --- | --- |')
    L.append(f'| Planform area | {tail_area * 1e4:.1f} cm² | {dors_area * 1e4:.1f} cm² |')
    L.append(f'| Chord × span | {chord * 1000:.0f} × {span * 1000:.0f} mm | {(dors["hi"][1] - dors["lo"][1]) * 1000:.0f} × {(dors["hi"][2] - dors["lo"][2]) * 1000:.0f} mm (quarter disc) |')
    L.append(f'| Thickness | {tail_t * 1000:.1f} mm | {dors_t * 1000:.1f} mm |')
    L.append(f'| Aspect ratio | {aspect:.2f} | – |')
    L.append(f'| Moves | yes, servo axis at CAD y = {hinge[1] * 1000:.0f} mm (vertical axis) | no (fixed) |')
    L.append(f'| Leading / trailing edge behind the axis | {le * 1000:.0f} / {te * 1000:.0f} mm | – |')
    L.append(f'| Center of pressure (¼ chord) | {tail_cp * 1000:.0f} mm behind the axis, height {(tail_lo[2] + tail_hi[2]) / 2 * 1000:.0f} mm | CAD {mm(dors_cp)} mm |')
    L.append(f'| Deflection in the exported CAD | {tail_deflection:.1f}° | – |\n')
    L.append(f'Tail servo: max angle ±{servo["max_angle_deg"]}°, speed {60 / servo["speed_s_per_60deg"]:.0f}°/s, torque {servo["torque_kgf_cm"] * 0.0981:.2f} N·m (D955TW at 7.4 V).\n')
    L.append('## Values for the Unity simulation (`RobotFishPhysics`)\n')
    L.append('| Field | Value |\n| --- | --- |')
    L.append(f'| `displacedVolume` | {vol_disp:.5f} m³ |')
    L.append(f'| `structureMass` | {m_dry:.3f} kg |')
    L.append(f'| `structureCenterOfMass` (Unity) | {mm(to_unity(com_dry))} mm |')
    L.append(f'| `structureInertia` (Unity x, y, z) | ({iu[0, 0]:.4f}, {iu[1, 1] + added_rot:.4f}, {iu[2, 2] + added_rot:.4f}) kg·m² |')
    L.append(f'| `movingMass`, `movingMassTravel`, `movingMassSpeed` | {m_move:.2f} kg, ±{half_travel:.3f} m, {mm_spec["speed_mm_s"] / 1000:.3f} m/s |')
    L.append(f'| `tailHinge` (Unity) | {mm(to_unity(hinge))} mm |')
    L.append(f'| tail fin center of pressure | {tail_cp:.3f} m behind the hinge, height {(tail_lo[2] + tail_hi[2]) / 2:.3f} m |')
    L.append(f'| `tailFinArea`, `dorsalFinArea` | {tail_area:.4f} m², {dors_area:.4f} m² |')
    L.append(f'| `dorsalFinCenter` (Unity) | {mm(to_unity(dors_cp))} mm |')
    L.append(f'| `tailServoSpeed`, `maxTailAngle` | {60 / servo["speed_s_per_60deg"]:.0f}°/s, {servo["max_angle_deg"]}° |')
    report = os.path.join(args.docs, 'RobotFish_CAD_Report.md')
    with open(report, 'w') as f:
        f.write('\n'.join(L) + '\n')
    print('\n'.join(L))
    print(f'\nwritten: {report}')
    draw(parts, os.path.join(args.docs, 'images'), com, cob, move_pos, half_travel)


# ---------------------------------------------------------------- images
GROUPS = [
    ('hull: tube and caps', ['tube', 'cap_front', 'cap_rear'], '#bdbdbd'),
    ('dome and camera', ['dome_cercle', 'camera', 'camera_mount'], '#6baed6'),
    ('electronics: RPi + Navigator, battery, trays, rails', ['tray_rpi', 'rpi_navigator', 'tray_battery', 'battery', 'rail_400', 'rail_400_1'], '#41ab5d'),
    ('sliding mass: servo HS-5496MH, rod, weight carriage', ['_hs_5496mh', 'conector_servo', 'conector_servo_1', 'haste', 'conector_pesos'], '#d62728'),
    ('tail drive: clamp, mount, servo D955TW, arm', ['clamp', 'servo_mount', 'underwater_servo', 'underwater_servo_hub', 'conector_calda'], '#756bb1'),
    ('tail fin', ['calda'], '#ff7f0e'),
    ('dorsal fin', ['calda_lateral'], '#8c564b'),
]


def draw(parts, out, com, cob, move_pos, half_travel):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from matplotlib.patches import Patch
    os.makedirs(out, exist_ok=True)
    fig, axes = plt.subplots(2, 1, figsize=(15, 9.5))
    for ax, (a, b, title, ylabel) in zip(axes, [(1, 2, 'Side view (CAD Y–Z): front/dome on the left, Z up', 'Z [mm]'),
                                                (1, 0, 'Top view (CAD Y–X)', 'X [mm]')]):
        for _, names, color in GROUPS:
            for n in names:
                tri = parts[n]['tri'][:, :, [a, b]] * 1000
                ax.add_collection(PolyCollection(tri, facecolor=color, edgecolor='none', alpha=0.35 if n == 'tube' else 0.85))
        if b == 2:
            ax.plot(com[1] * 1000, com[2] * 1000, 'k+', ms=14, mew=2)
            ax.annotate('CoM', (com[1] * 1000, com[2] * 1000), xytext=(8, -14), textcoords='offset points')
            ax.plot(cob[1] * 1000, cob[2] * 1000, 'bx', ms=10, mew=2)
            ax.annotate('CoB', (cob[1] * 1000, cob[2] * 1000), xytext=(8, 6), textcoords='offset points', color='b')
            ax.plot([(move_pos[1] - half_travel) * 1000, (move_pos[1] + half_travel) * 1000], [move_pos[2] * 1000] * 2, '-', color='#444', lw=3)
            ax.plot(move_pos[1] * 1000, move_pos[2] * 1000, 's', color='#444', ms=7)
            ax.annotate('moving mass and its travel', (move_pos[1] * 1000, move_pos[2] * 1000), xytext=(-60, 14), textcoords='offset points', fontsize=8)
        ax.autoscale()
        ax.set_aspect('equal')
        ax.set_title(title)
        ax.set_xlabel('Y [mm]')
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.3)
    axes[0].legend(handles=[Patch(color=c, label=l) for l, _, c in GROUPS], fontsize=8, loc='upper right', ncol=2)
    plt.tight_layout()
    path = os.path.join(out, 'robot_fish_views.png')
    plt.savefig(path, dpi=80)
    print('written:', path)


if __name__ == '__main__':
    main()
