"""Thin wrappers around the `ign` CLI for spawning, removing and querying models.

Used by the box spawner and the pick-and-place node. In a real cell these calls
don't exist: they only stand in for physics we don't simulate exactly (gripping).
"""
import re
import subprocess

WORLD = 'factory'

BOX_SIZE = (0.12, 0.12, 0.08)
BOX_MASS = 0.5


CONVEYOR_TOPIC = '/conveyor/box_cmd_vel'


def box_sdf(name, size=BOX_SIZE, mass=BOX_MASS, on_belt=False):
    """on_belt=True adds a VelocityControl plugin so the box rides the conveyor."""
    sx, sy, sz = size
    ixx = mass * (sy * sy + sz * sz) / 12
    iyy = mass * (sx * sx + sz * sz) / 12
    izz = mass * (sx * sx + sy * sy) / 12
    return (
        f'<sdf version="1.8"><model name="{name}"><link name="link">'
        f'<inertial><mass>{mass}</mass><inertia><ixx>{ixx:.6f}</ixx><iyy>{iyy:.6f}</iyy>'
        f'<izz>{izz:.6f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
        f'<collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
        f'<surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision>'
        f'<visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
        f'<material><ambient>0.9 0.35 0.05 1</ambient><diffuse>0.9 0.35 0.05 1</diffuse></material>'
        f'</visual></link>'
        + (f'<plugin filename="ignition-gazebo-velocity-control-system" '
           f'name="ignition::gazebo::systems::VelocityControl"><topic>{CONVEYOR_TOPIC}</topic></plugin>'
           if on_belt else '')
        + '</model></sdf>'
    )


def _service(srv, reqtype, req, timeout_ms=5000):
    cmd = ['ign', 'service', '-s', f'/world/{WORLD}/{srv}',
           '--reqtype', reqtype, '--reptype', 'ignition.msgs.Boolean',
           '--timeout', str(timeout_ms), '--req', req]
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_ms / 1000 + 5)
    return 'data: true' in out.stdout


def spawn(name, sdf, xyz, quat=(0.0, 0.0, 0.0, 1.0)):
    x, y, z = xyz
    qx, qy, qz, qw = quat
    req = (f"sdf: '{sdf}' name: '{name}' allow_renaming: false "
           f"pose: {{position: {{x: {x} y: {y} z: {z}}} "
           f"orientation: {{x: {qx} y: {qy} z: {qz} w: {qw}}}}}")
    return _service('create', 'ignition.msgs.EntityFactory', req)


def remove(name):
    return _service('remove', 'ignition.msgs.Entity', f'name: "{name}" type: MODEL')


def publish_empty(topic):
    subprocess.run(['ign', 'topic', '-t', topic, '-m', 'ignition.msgs.Empty', '-p', ' '],
                   capture_output=True, timeout=10)


_BLOCK = re.compile(r'^pose \{(.*?)^\}', re.S | re.M)
_NUM = r'(-?[0-9.]+(?:e[-+]?\d+)?)'


def _vec(block, section, keys, default):
    m = re.search(section + r' \{([^}]*)\}', block)
    if not m:
        return default
    vals = []
    for k in keys:
        f = re.search(r'\b' + k + r': ' + _NUM, m.group(1))
        vals.append(float(f.group(1)) if f else 0.0)   # protobuf omits zero fields
    return tuple(vals)


def model_poses(prefix=''):
    """Return {name: ((x,y,z),(qx,qy,qz,qw))} for models whose name starts with prefix."""
    out = subprocess.run(['ign', 'topic', '-e', '-n', '1', '-t', f'/world/{WORLD}/pose/info'],
                         capture_output=True, text=True, timeout=10).stdout
    poses = {}
    for block in _BLOCK.findall(out):
        nm = re.search(r'name: "([^"]+)"', block)
        if not nm or not nm.group(1).startswith(prefix):
            continue
        p = _vec(block, 'position', 'xyz', (0.0, 0.0, 0.0))
        q = _vec(block, 'orientation', ('x', 'y', 'z', 'w'), (0.0, 0.0, 0.0, 1.0))
        poses.setdefault(nm.group(1), (p, q))
    return poses
