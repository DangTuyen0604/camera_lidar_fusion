#!/usr/bin/env python3
"""
Download third-party warehouse assets and convert them for MuJoCo.

Converts COLLADA meshes to per-texture OBJ parts in metres, and writes them
with their textures into src/warehouse_simulation/models/<model>/. The
hand-written model.sdf files reference these parts as visuals only; every
model keeps its primitive <collision>, which the LiDAR, the static map and
physics use, so the navigation stack is unaffected by the visual detail.

Sources (all free to redistribute):
  AWS RoboMaker Small Warehouse World (MIT-0): shelf frame, pallet jack,
      dumpster, floor and wall textures
  Gazebo Fuel, OpenRobotics (CC0): Walking person, Standing person,
      Casual female
  ambientCG (CC0): Cardboard002, Wood049

The results are committed, so this only needs re-running to change assets.
Needs: pip install trimesh pycollada pillow

Usage:
    python3 tools/import_warehouse_assets.py [--cache ~/.cache/warehouse_assets]
"""

import argparse
import io
from pathlib import Path
import urllib.parse
import urllib.request
import zipfile

import numpy as np
from PIL import Image
import trimesh
from trimesh.resolvers import FilePathResolver

ROOT = Path(__file__).resolve().parents[1]
MODELS = ROOT / 'src/warehouse_simulation/models'
WORLD_TEXTURES = ROOT / 'src/warehouse_simulation/worlds/textures'
AWS_ZIP = ('https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/'
           'archive/refs/heads/ros2.zip')
FUEL = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/{}.zip'
AMBIENTCG = 'https://ambientcg.com/get?file={}_1K-JPG.zip'
MAX_TEXTURE = 1024
# Hi-vis orange (RGB) multiplied into the shirt texture, keeping its shading.
HI_VIS = np.array([1.0, 0.45, 0.05])


def fetch_zip(url, target):
    if not target.exists():
        print(f'downloading {url}')
        # ambientCG rejects urllib's default User-Agent (HTTP 403).
        request = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
        with urllib.request.urlopen(request) as response:
            zipfile.ZipFile(io.BytesIO(response.read())).extractall(target)
    return target


def load_parts(dae):
    """Return the textured sub-meshes of a COLLADA file (scene units)."""
    scene = trimesh.load(str(dae), force='scene',
                         resolver=FilePathResolver(str(dae.parent), allow_anywhere=True))
    return scene.dump(concatenate=False)


def texture_of(part):
    material = part.visual.material
    image = getattr(material, 'baseColorTexture', None)
    if image is None:
        image = getattr(material, 'image', None)
    return image


def save_texture(image, path, tint=None):
    image = image.convert('RGB')
    image.thumbnail((MAX_TEXTURE, MAX_TEXTURE))
    if tint is not None:
        pixels = np.asarray(image, dtype=np.float32) / 255.0
        luminance = pixels.mean(axis=2, keepdims=True)
        # Keep the fabric shading but lift the darkest folds: a plain
        # multiply turns shadowed texels black instead of dark orange.
        pixels = np.clip(0.45 + 0.9 * luminance, 0, 1) * tint
        image = Image.fromarray((pixels * 255).astype(np.uint8))
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)


def export_model(name, dae, *, size=None, height=None, keep=None, tint=None,
                 tint_parts=()):
    """Write meshes/<name>_<i>.obj + materials/textures/<name>_<i>.png.

    size: fit the model's bounding box to (x, y, z) metres (non-uniform);
    height: uniform scale to this height. The result is centred in x/y and
    rests on z=0. keep: indices of the parts to export.
    """
    parts = load_parts(dae)
    if keep is not None:
        parts = [parts[i] for i in keep]
    bounds = trimesh.util.concatenate(parts).bounds
    extent = bounds[1] - bounds[0]
    if size is not None:
        scale = np.asarray(size) / extent
    else:
        scale = np.full(3, height / extent[2])
    offset = -np.array([(bounds[0][0] + bounds[1][0]) / 2,
                        (bounds[0][1] + bounds[1][1]) / 2, bounds[0][2]])
    out = MODELS / name
    written = []
    for index, part in enumerate(parts):
        part = part.copy()
        part.apply_translation(offset)
        part.apply_transform(np.diag([*scale, 1.0]))
        stem = f'{name}_{index}'
        (out / 'meshes').mkdir(parents=True, exist_ok=True)
        part.export(out / 'meshes' / f'{stem}.obj', include_texture=False)
        image = texture_of(part)
        if image is not None:
            save_texture(image, out / 'materials' / 'textures' / f'{stem}.png',
                         tint if index in tint_parts else None)
        written.append((stem, image is not None))
    print(f'{name}: {len(written)} parts, size {np.round(extent * scale, 2)}')
    return written


def parts_textured_with(dae, keywords):
    """Indices of parts whose texture file name contains one of keywords.

    Material names are empty in these files, so each part's texture image is
    matched against the image files next to the model.
    """
    textures = {}
    for path in (dae.parent.parent / 'materials' / 'textures').glob('*'):
        if path.suffix.lower() in ('.png', '.jpg'):
            textures[path.name.lower()] = np.asarray(Image.open(path).convert('RGB'))[::64, ::64]
    indices = []
    for index, part in enumerate(load_parts(dae)):
        image = texture_of(part)
        if image is None:
            continue
        sample = np.asarray(image.convert('RGB'))[::64, ::64]
        names = [n for n, t in textures.items() if t.shape == sample.shape
                 and np.array_equal(t, sample)]
        if any(k in n for n in names for k in keywords):
            indices.append(index)
    return indices


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--cache', type=Path,
                        default=Path.home() / '.cache' / 'warehouse_assets')
    args = parser.parse_args()
    cache = args.cache.expanduser()
    cache.mkdir(parents=True, exist_ok=True)

    aws = fetch_zip(AWS_ZIP, cache / 'aws') / 'aws-robomaker-small-warehouse-world-ros2'
    aws_models = aws / 'models'

    def aws_dae(name):
        return (aws_models / f'aws_robomaker_warehouse_{name}' / 'meshes'
                / f'aws_robomaker_warehouse_{name}_visual.DAE')

    # Shelf: part 0 is the bare frame; the goods (parts 1-3) are merged
    # meshes that cannot be labelled per box, so labelled crates are placed
    # by model.sdf instead. Fitted to the existing 3.0 x 0.7 x 2.4 m footprint.
    export_model('shelf', aws_dae('ShelfD_01'), size=(3.0, 0.7, 2.4), keep=[0])
    export_model('pallet_jack', aws_dae('PalletJackB_01'), height=1.0)
    export_model('dumpster', aws_dae('TrashCanC_01'), height=1.27)

    for fuel_name, model, height, clothing in (
            ('Walking person', 'worker', 1.75, ('tshirt', 'jeans')),
            ('Standing person', 'worker_standing', 1.78, ('tshirt', 'jeans')),
            ('Casual female', 'worker_female', 1.65, ('casualsuit',))):
        # Hi-vis top and trousers: the orange must reach down through the
        # LiDAR plane (~0.45 m) so camera blobs and scan points coincide.
        root = fetch_zip(FUEL.format(urllib.parse.quote(fuel_name)),
                         cache / fuel_name.replace(' ', '_'))
        dae = next(root.rglob('*.dae'))
        export_model(model, dae, height=height, tint=HI_VIS,
                     tint_parts=parts_textured_with(dae, clothing))

    for asset, model, name in (('Cardboard002', 'cargo_box', 'cardboard'),
                               ('Wood049', 'pallet', 'wood')):
        root = fetch_zip(AMBIENTCG.format(asset), cache / asset)
        color = next(root.glob('*_Color.jpg'))
        save_texture(Image.open(color), MODELS / model / 'materials' / 'textures' / f'{name}.png')

    for source, name in (('GroundB_01', 'floor'), ('WallB_01', 'wall')):
        texture = (aws_models / f'aws_robomaker_warehouse_{source}' / 'materials'
                   / 'textures' / f'aws_robomaker_warehouse_{source}.png')
        save_texture(Image.open(texture), WORLD_TEXTURES / f'{name}.png')

    (MODELS / 'THIRD_PARTY_ASSETS.md').write_text(
        '# Third-party assets\n\n'
        'Generated by tools/import_warehouse_assets.py.\n\n'
        '- shelf, pallet_jack, dumpster meshes; worlds/textures/floor.png, wall.png:\n'
        '  AWS RoboMaker Small Warehouse World, MIT-0\n'
        '  (https://github.com/aws-robotics/aws-robomaker-small-warehouse-world).\n'
        '  Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.\n'
        '  Permission is hereby granted, free of charge, to any person obtaining a\n'
        '  copy of this software, to deal in the Software without restriction.\n'
        '- worker, worker_standing, worker_female: Gazebo Fuel OpenRobotics\n'
        '  "Walking person", "Standing person", "Casual female", CC0 1.0\n'
        '  (clothing textures tinted hi-vis orange).\n'
        '- cargo_box cardboard.png (Cardboard002), pallet wood.png (Wood049):\n'
        '  ambientCG, CC0 1.0 (https://ambientcg.com).\n')


if __name__ == '__main__':
    main()
