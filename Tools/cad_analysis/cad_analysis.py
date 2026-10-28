#!/usr/bin/env python3
"""Physical properties of the robot fish estimated from its CAD model (Onshape URDF + STL meshes).

What it does:
  1. Reads the URDF, places every part in the robot frame (forward kinematics, all joints at 0).
  2. Measures each part from its mesh: volume, centroid, inertia (exact integration over the closed mesh).
  3. Gives each part a mass from assumptions.yaml (CAD volume × density, or a datasheet mass).
  4. Computes the displaced volume, centers of mass and buoyancy, trim ballast, pitch authority of the
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

    # ---- 3. trim ballast that makes the robot neutral (or the target buoyancy) and level
    trim = A['trim_ballast']
    m_total = RHO_WATER * vol_disp / (1 + trim['target_buoyancy_percent'] / 100)
    m_trim = m_total - m_dry
    y_trim = (m_total * cob[1] - m_dry * com_dry[1]) / m_trim     # CoM right below the CoB → rests level
    trim_pos = np.array([0, y_trim, trim['height_mm'] / 1000])
    rows.append(('trim ballast (lead)', 'computed', 'estimate', 0.0, m_trim, trim_pos, np.zeros((3, 3))))
    com = sum(r[4] * r[5] for r in rows) / m_total
    bg = cob[2] - com[2]                                          # CoM below CoB

    # ---- 4. inertia about the CoM (parallel-axis theorem) + added inertia of the hull in pitch and yaw
    inertia = np.zeros((3, 3))
    for r in rows:
        d = r[5] - com
        inertia += r[6] + r[4] * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    added_rot = RHO_WATER * np.pi * r_tube**2 * tube_len**3 / 12  # slender cylinder, about a lateral axis
    added_lat = RHO_WATER * np.pi * r_tube**2 * tube_len          # added mass sideways / vertical

    # ---- 5. sliding mass: pitch authority
    sm = A['sliding_mass']
    m_slide = A['extra_masses']['sliding_weights']['mass'] / 1000
    travel = sm['horn_radius_mm'] / 1000 * np.sin(np.radians(sm['servo_range_deg']))
    restoring = m_total * G * bg                                  # N·m per radian of pitch
    pitch_max = np.degrees(m_slide * G * travel / restoring)
    carriage_speed = sm['horn_radius_mm'] / 1000 * np.radians(60) / sm['servo_speed_s_per_60deg']

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
    L.append(f'| **Total** | | | | **{m_total * 1000:.0f}** | |\n')
    L.append('## Buoyancy and balance\n')
    L.append('| Quantity | Value |\n| --- | --- |')
    L.append(f'| Displaced volume | {vol_disp * 1e3:.2f} L (' + ', '.join(f'{n} {v * 1e3:.2f}' for n, v, _ in hull[:3]) + f', parts outside the hull {sum(v for _, v, _ in hull[3:]) * 1e3:.2f}) |')
    L.append(f'| Dry mass (without trim ballast) | {m_dry:.2f} kg → buoyancy {(RHO_WATER * vol_disp / m_dry - 1) * 100:+.0f} % of the weight |')
    L.append(f'| Target buoyancy | {trim["target_buoyancy_percent"]:+g} % |')
    L.append(f'| Trim ballast needed | **{m_trim * 1000:.0f} g** at y = {y_trim * 1000:.1f} mm, z = {trim["height_mm"]} mm (CAD) |')
    L.append(f'| Total mass | {m_total:.2f} kg |')
    L.append(f'| Center of buoyancy (CAD) | {mm(cob)} mm |')
    L.append(f'| Center of mass (CAD) | {mm(com)} mm |')
    L.append(f'| CoM below CoB (BG) | {bg * 1000:.1f} mm → righting moment {restoring:.2f} N·m/rad |\n')
    L.append('## Sliding mass (pitch)\n')
    L.append('| Quantity | Value |\n| --- | --- |')
    L.append(f'| Moving weight | {m_slide * 1000:.0f} g |')
    L.append(f'| Travel | ±{travel * 1000:.1f} mm (horn {sm["horn_radius_mm"]} mm × sin {sm["servo_range_deg"]}°) |')
    L.append(f'| Carriage speed | {carriage_speed:.2f} m/s (HS-5496MH {sm["servo_speed_s_per_60deg"]} s/60°) |')
    L.append(f'| CoM shift at full travel | ±{m_slide * travel / m_total * 1000:.2f} mm |')
    L.append(f'| Maximum static pitch | ±{pitch_max:.1f}° |')
    for mw in (0.8, 1.2):
        L.append(f'| (with {mw * 1000:.0f} g in the carriage) | ±{np.degrees(mw * G * travel / restoring):.1f}° |')
    L.append('')
    L.append('## Inertia\n')
    iu = to_unity_tensor(inertia)
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
    L.append(f'| `centerOfMass` (Unity) | {mm(to_unity(com))} mm |')
    L.append(f'| `inertia` (Unity x, y, z) | ({iu[0, 0]:.4f}, {iu[1, 1] + added_rot:.4f}, {iu[2, 2] + added_rot:.4f}) kg·m² |')
    L.append(f'| `slidingMass`, `slidingTravel`, `slidingSpeed` | {m_slide:.2f} kg, {travel:.3f} m, {carriage_speed:.2f} m/s |')
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
    draw(parts, os.path.join(args.docs, 'images'), com, cob, trim_pos)


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


def draw(parts, out, com, cob, trim_pos):
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
            ax.plot(trim_pos[1] * 1000, trim_pos[2] * 1000, 's', color='#444', ms=6)
            ax.annotate('trim lead (estimate)', (trim_pos[1] * 1000, trim_pos[2] * 1000), xytext=(-40, -24), textcoords='offset points', fontsize=8)
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
