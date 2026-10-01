"""Convert the warehouse SDF world (primitives and OBJ meshes) into MuJoCo spec geometry."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

# Geom groups: the MuJoCo viewer and renderers show 0-2 by default.  Visuals
# (including detailed meshes) are group 0 for the cameras; the primitive
# collisions are group 3, which the simulated LiDAR ray-casts so its returns
# match the static map generated from the same collision geometry.
VISUAL_GROUP = 0
COLLISION_GROUP = 3
# MuJoCo uses the larger friction of a geom pair, Gazebo/ODE the smaller.  A
# low world value lets the robot's own coefficients decide, reproducing the
# Gazebo pairs (see mjcf/amr.xml).
WORLD_FRICTION = [0.2, 0.005, 0.0001]
# Contact bit 2 is the floor: the robot's casters (mjcf/amr.xml) collide only
# with it, never with obstacles.
FLOOR_BITS = 1 | 2


def parse_pose(text):
    """Return (pos, quat_wxyz) for an SDF 'x y z roll pitch yaw' pose."""
    values = [float(v) for v in (text or '').split()] if text else []
    values += [0.0] * (6 - len(values))
    x, y, z, roll, pitch, yaw = values[:6]
    return np.array([x, y, z]), euler_to_quat(roll, pitch, yaw)


def euler_to_quat(roll, pitch, yaw):
    """SDF fixed-axis XYZ rotation (R = Rz Ry Rx) as a wxyz quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def compose(parent, child):
    """Compose two (pos, quat) transforms: parent * child."""
    ppos, pquat = parent
    cpos, cquat = child
    rotated = np.zeros(3)
    mujoco.mju_rotVecQuat(rotated, cpos, pquat)
    quat = np.zeros(4)
    mujoco.mju_mulQuat(quat, pquat, cquat)
    return ppos + rotated, quat


IDENTITY = (np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))


def _pose_of(element):
    node = element.find('pose')
    return parse_pose(node.text) if node is not None else IDENTITY


def _rgba(visual):
    material = visual.find('material')
    if material is not None:
        for tag in ('diffuse', 'ambient'):
            node = material.find(tag)
            if node is not None and node.text:
                values = [float(v) for v in node.text.split()]
                return (values + [1.0] * 4)[:4]
    return [0.6, 0.6, 0.6, 1.0]


def _geometry(element):
    """Return (mjtGeom, size) for an SDF <geometry>, or None if unsupported.

    For a <mesh> the size is the mesh <uri> text instead.
    """
    geometry = element.find('geometry')
    if geometry is None:
        return None
    mesh = geometry.find('mesh')
    if mesh is not None:
        return mujoco.mjtGeom.mjGEOM_MESH, mesh.find('uri').text.strip()
    box = geometry.find('box')
    if box is not None:
        sx, sy, sz = (float(v) for v in box.find('size').text.split())
        return mujoco.mjtGeom.mjGEOM_BOX, [sx / 2, sy / 2, sz / 2]
    cylinder = geometry.find('cylinder')
    if cylinder is not None:
        radius = float(cylinder.find('radius').text)
        length = float(cylinder.find('length').text)
        return mujoco.mjtGeom.mjGEOM_CYLINDER, [radius, length / 2, 0]
    sphere = geometry.find('sphere')
    if sphere is not None:
        return mujoco.mjtGeom.mjGEOM_SPHERE, [float(sphere.find('radius').text), 0, 0]
    plane = geometry.find('plane')
    if plane is not None:
        size_node = plane.find('size')
        sx, sy = ((float(v) for v in size_node.text.split())
                  if size_node is not None else (100.0, 100.0))
        return mujoco.mjtGeom.mjGEOM_PLANE, [sx / 2, sy / 2, 1.0]
    return None


def _resolve(uri, base_dir, model_paths):
    """Absolute path of a relative or model:// asset URI."""
    if uri.startswith('model://'):
        name, _, rest = uri[len('model://'):].partition('/')
        for directory in model_paths:
            candidate = Path(directory) / name / rest
            if candidate.is_file():
                return candidate.resolve()
        raise FileNotFoundError(f'Cannot resolve {uri} in {list(model_paths)}')
    path = Path(base_dir or '.') / uri
    if not path.is_file():
        raise FileNotFoundError(f'Asset {uri} not found in {base_dir}')
    return path.resolve()


# Assets already added to each spec, keyed by absolute file path, so a model
# included several times (shelves, entity pools) shares one mesh/texture.
_ASSET_NAMES = {}


def _asset(spec, kind, path):
    names = _ASSET_NAMES.setdefault(id(spec), {})
    key = (kind, str(path))
    if key in names:
        return names[key]
    name = f'{kind}{len(names)}_{path.stem}'
    if kind == 'mesh':
        mesh = spec.add_mesh()
        mesh.name, mesh.file = name, str(path)
        # Visual meshes are open surfaces; a shell inertia never fails on a
        # non-watertight mesh (they carry no mass anyway, see density=0).
        mesh.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL
    else:
        texture = spec.add_texture()
        texture.name, texture.file = name, str(path)
        texture.type = mujoco.mjtTexture.mjTEXTURE_2D
    names[key] = name
    return name


def _material(spec, visual, geom_type, base_dir, model_paths):
    """Name of a textured material for a visual with <albedo_map>, else None."""
    material = visual.find('material')
    albedo = material.find('.//albedo_map') if material is not None else None
    if albedo is None or not albedo.text:
        return None
    if spec is None:
        raise ValueError('textured visuals need the MjSpec (pass spec=)')
    texture = _asset(spec, 'tex', _resolve(albedo.text.strip(), base_dir, model_paths))
    repeat = material.find('texrepeat')  # MuJoCo-only: tiles per metre
    key = (texture, repeat.text if repeat is not None else '')
    names = _ASSET_NAMES.setdefault(id(spec), {})
    if ('mat', key) not in names:
        mat = spec.add_material()
        mat.name = f'mat{len(names)}_{texture}'
        mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = texture
        if geom_type != mujoco.mjtGeom.mjGEOM_MESH:
            # Primitives have no UVs: tile the texture in world units.
            mat.texuniform = True
            mat.texrepeat = ([float(v) for v in repeat.text.split()]
                             if repeat is not None else [1.0, 1.0])
        names[('mat', key)] = mat.name
    return names[('mat', key)]


def add_model_geoms(body, model, frame=IDENTITY, prefix='', collision_bits=1,
                    spec=None, base_dir=None, model_paths=()):
    """Add every link visual/collision geometry of an SDF <model> to body.

    Meshes and textures are resolved against base_dir (the model's folder)
    or model_paths for model:// URIs and need spec to register the assets.
    """
    count = 0
    for link in model.findall('link'):
        link_frame = compose(frame, _pose_of(link))
        for tag in ('visual', 'collision'):
            for element in link.findall(tag):
                geometry = _geometry(element)
                if geometry is None:
                    continue
                geom_type, size = geometry
                if geom_type == mujoco.mjtGeom.mjGEOM_MESH and tag == 'collision':
                    continue  # collisions stay primitive (LiDAR, map, physics)
                pos, quat = compose(link_frame, _pose_of(element))
                geom = body.add_geom()
                geom.name = f"{prefix}{link.get('name')}/{element.get('name')}"
                geom.type = geom_type
                if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
                    if spec is None:
                        raise ValueError('mesh visuals need the MjSpec (pass spec=)')
                    geom.meshname = _asset(spec, 'mesh', _resolve(size, base_dir, model_paths))
                    geom.density = 0.0
                else:
                    geom.size = size
                geom.pos = pos
                geom.quat = quat
                if tag == 'visual':
                    material = _material(spec, element, geom_type, base_dir, model_paths)
                    if material is not None:
                        geom.material = material
                        geom.rgba = [1.0, 1.0, 1.0, 1.0]  # texture colours as-is
                    else:
                        geom.rgba = _rgba(element)
                    geom.contype = 0
                    geom.conaffinity = 0
                    geom.group = VISUAL_GROUP
                else:
                    geom.rgba = [0.2, 0.8, 0.2, 0.3]
                    geom.friction = WORLD_FRICTION
                    is_floor = geom_type == mujoco.mjtGeom.mjGEOM_PLANE
                    bits = FLOOR_BITS if is_floor else collision_bits
                    geom.contype = bits
                    geom.conaffinity = bits
                    geom.group = COLLISION_GROUP
                count += 1
    return count


def find_model_file(uri, model_paths):
    """Resolve a model:// URI against a list of model directories."""
    name = uri.replace('model://', '').strip('/')
    for directory in model_paths:
        candidate = Path(directory) / name / 'model.sdf'
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f'Cannot resolve {uri} in {list(model_paths)}')


def model_element(sdf_text):
    """Return the first <model> element of an SDF document string."""
    root = ET.fromstring(sdf_text)
    model = root if root.tag == 'model' else root.find('.//model')
    if model is None:
        raise ValueError('SDF document has no <model>')
    return model


def add_world(spec, world_file, model_paths):
    """Add all models of an SDF world to spec.worldbody as static geometry."""
    root = ET.parse(world_file).getroot()
    world = root.find('world') if root.tag == 'sdf' else root
    model_paths = [Path(world_file).parent, *model_paths]
    added = []
    for model in world.findall('model'):
        name = model.get('name')
        add_model_geoms(spec.worldbody, model, _pose_of(model), prefix=f'{name}/',
                        spec=spec, base_dir=Path(world_file).parent,
                        model_paths=model_paths)
        added.append(name)
    for include in world.findall('include'):
        uri = include.find('uri').text.strip()
        path = find_model_file(uri, model_paths)
        model = model_element(path.read_text())
        name_node = include.find('name')
        name = name_node.text.strip() if name_node is not None else model.get('name')
        add_model_geoms(spec.worldbody, model, _pose_of(include), prefix=f'{name}/',
                        spec=spec, base_dir=path.parent, model_paths=model_paths)
        added.append(name)
    return added
