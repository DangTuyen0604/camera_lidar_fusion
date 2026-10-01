#!/usr/bin/env python3
"""
Render auto-labelled YOLO training images from the MuJoCo warehouse.

Each frame places the robot at a random free pose, scatters boxes, pallets
and workers in front of its camera, randomises lighting, colours and camera
mounting, and renders the front camera. Labels come from MuJoCo's
segmentation render, so every box is the visible (occlusion-aware) extent of
the object. Crates stocked on the shelves are labelled as boxes too;
otherwise the model would learn that boxes are background.

Output is a flat YOLO folder (images/, labels/, data.yaml) ready to upload
to Roboflow, which does the train/valid/test split. The images are varied
rather than photoreal: mix them with real images and validate on real ones.

Usage:
    python3 tools/generate_synthetic_dataset.py --frames 2000 --out ~/synthetic_dataset
    python3 tools/generate_synthetic_dataset.py --frames 20 --out ~/synthetic_test --preview
        # quick look: labels drawn in ~/synthetic_test_preview/
"""

import argparse
import colorsys
import math
from pathlib import Path
import sys

import cv2
import mujoco
import numpy as np
from scipy.ndimage import distance_transform_edt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/warehouse_mujoco'))
from warehouse_mujoco.simulator import WarehouseSim, yaw_to_quat  # noqa: E402

MODELS = ROOT / 'src/warehouse_simulation/models'
WORLD = ROOT / 'src/warehouse_simulation/worlds/warehouse.sdf'
SCENE = ROOT / 'src/warehouse_mujoco/mjcf/scene.xml'
MESHES = ROOT / 'src/openamrobot_description/meshes/visual'
MAP = ROOT / 'src/navigation_bringup/maps/warehouse_map.pgm'
MAP_ORIGIN, MAP_RESOLUTION = (-9.0, -7.0), 0.05

# Same ids as the benchmark ground truth (scenario_runner.py).
CLASSES = ['person', 'box', 'pallet']
WORKERS = ('worker', 'worker_standing', 'worker_female')
ENTITY_CLASS = {'cargo_box': 1, 'pallet': 2, **{w: 0 for w in WORKERS}}
# Unlabelled look-alikes the model must learn to ignore.
DISTRACTORS = ('pallet_jack', 'dumpster')
# Spawn heights used by scenarios.yaml; a box on a pallet sits on its deck.
SPAWN_Z = {'cargo_box': 0.25, 'pallet': 0.10, 'person': 0.0,
           'pallet_jack': 0.0, 'dumpster': 0.0}
BOX_ON_PALLET_Z = 0.44
# Clearance radius (m) and per-frame maximum of each kind of object.
FOOTPRINT = {'cargo_box': 0.40, 'pallet': 0.75, 'person': 0.40,
             'pallet_jack': 0.65, 'dumpster': 0.85}
MAX_COUNT = {'cargo_box': 6, 'pallet': 3, 'person': 2, 'pallet_jack': 1, 'dumpster': 1}
WIDTH, HEIGHT = 640, 480
MIN_PIXELS, MIN_SIDE = 150, 10


def free_space():
    """Distance (m) from each map cell to the nearest static obstacle."""
    image = cv2.imread(str(MAP), cv2.IMREAD_GRAYSCALE)
    free = np.flipud(image > 200)  # row 0 = lowest y, like the map origin
    return distance_transform_edt(free) * MAP_RESOLUTION


def clearance(dist, x, y):
    i = int((x - MAP_ORIGIN[0]) / MAP_RESOLUTION)
    j = int((y - MAP_ORIGIN[1]) / MAP_RESOLUTION)
    if 0 <= j < dist.shape[0] and 0 <= i < dist.shape[1]:
        return dist[j, i]
    return 0.0


class Generator:

    def __init__(self, seed):
        self.rng = np.random.default_rng(seed)
        entity_models = {name: MODELS / name / 'model.sdf'
                         for name in (*ENTITY_CLASS, *DISTRACTORS)}
        self.sim = WarehouseSim(SCENE, WORLD, [str(MODELS)], MESHES, entity_models,
                                pool_size=max(MAX_COUNT.values()), seed=seed)
        model = self.sim.model
        model.vis.map.znear = 0.02 / model.stat.extent  # see ros_node.run()
        self.renderer = mujoco.Renderer(model, HEIGHT, WIDTH)
        # Hide the robot's own chassis (visual group 1): the camera sits on
        # its nose and would otherwise fill the image bottom with it, which
        # no real training image contains.
        self.view = mujoco.MjvOption()
        self.view.geomgroup[:] = [1, 0, 1, 0, 0, 0]
        self.dist = free_space()
        self.camera = model.camera('front_camera').id
        self.cam_pos0 = model.cam_pos[self.camera].copy()
        self.cam_quat0 = model.cam_quat[self.camera].copy()
        self.hfov = 2 * math.atan(math.tan(math.radians(model.cam_fovy[self.camera]) / 2)
                                  * WIDTH / HEIGHT)
        self.rgba0 = model.geom_rgba.copy()
        self.light0 = model.light_diffuse.copy()
        self.headlight0 = model.vis.headlight.ambient.copy()
        # geom id -> instance key; entity slots are one instance each, every
        # shelf crate its own instance.
        self.geom_instance = {}
        for model_name, slots in self.sim._pools.items():
            if model_name not in ENTITY_CLASS:
                continue  # distractors occlude but are never labelled
            for body in slots:
                body_id = model.body(body).id
                for geom in np.nonzero(model.geom_bodyid == body_id)[0]:
                    self.geom_instance[int(geom)] = body
        for geom in range(model.ngeom):
            name = model.geom(geom).name or ''
            if '/stock_' in name:
                self.geom_instance[geom] = f'crate:{geom}'
        self.body_model = {body: name for name, slots in self.sim._pools.items()
                           for body in slots}

    # ------------------------------------------------------------ randomise
    def _robot_pose(self):
        while True:
            x = self.rng.uniform(-8.5, 8.5)
            y = self.rng.uniform(-6.5, 6.5)
            yaw = self.rng.uniform(-math.pi, math.pi)
            # Skip poses staring into a wall: need 3 m of open floor ahead.
            if clearance(self.dist, x, y) > 0.5 and all(
                    clearance(self.dist, x + d * math.cos(yaw), y + d * math.sin(yaw)) > 0.1
                    for d in np.arange(0.5, 3.01, 0.25)):
                return x, y, yaw

    def _place_entities(self, rx, ry, ryaw):
        placed = []  # (x, y, radius)
        pallets = []
        for kind in ('pallet', 'cargo_box', 'person', 'pallet_jack', 'dumpster'):
            for index in range(self.rng.integers(0, MAX_COUNT[kind] + 1)):
                model_name = WORKERS[self.rng.integers(len(WORKERS))] if kind == 'person' else kind
                name = f'{kind}_{index}'
                radius = FOOTPRINT[kind]
                if model_name == 'cargo_box' and pallets and self.rng.random() < 0.4:
                    x, y, yaw = pallets.pop()  # load a pallet: realistic stack
                    self.sim.spawn_entity(name, model_name, [x, y, BOX_ON_PALLET_Z],
                                          yaw_to_quat(yaw + self.rng.normal(0, 0.1)))
                    continue
                for _ in range(30):
                    bearing = ryaw + self.rng.uniform(-0.65, 0.65) * self.hfov
                    distance = self.rng.uniform(1.5, 8.0)
                    x = rx + distance * math.cos(bearing)
                    y = ry + distance * math.sin(bearing)
                    if (clearance(self.dist, x, y) > radius and all(
                            math.hypot(x - px, y - py) > radius + pr for px, py, pr in placed)):
                        break
                else:
                    continue
                yaw = self.rng.uniform(-math.pi, math.pi)
                placed.append((x, y, radius))
                if model_name == 'pallet':
                    pallets.append((x, y, yaw))
                self.sim.spawn_entity(name, model_name, [x, y, SPAWN_Z[kind]],
                                      yaw_to_quat(yaw))

    def _appearance(self):
        model, rng = self.sim.model, self.rng
        rgba = self.rgba0.copy()
        for geom in range(model.ngeom):
            h, s, v = colorsys.rgb_to_hsv(*np.clip(rgba[geom, :3], 0, 1))
            h = (h + rng.normal(0, 0.03)) % 1.0
            s = np.clip(s * rng.uniform(0.7, 1.3), 0, 1)
            v = np.clip(v * rng.uniform(0.75, 1.25), 0, 1)
            rgba[geom, :3] = colorsys.hsv_to_rgb(h, s, v)
        model.geom_rgba[:] = rgba
        model.light_diffuse[:] = self.light0 * rng.uniform(0.4, 1.6)
        model.vis.headlight.ambient[:] = self.headlight0 * rng.uniform(0.5, 1.5)
        # Mounting tolerance: +/-5 cm height, +/-6 deg pitch.
        model.cam_pos[self.camera] = self.cam_pos0 + [0, 0, rng.uniform(-0.05, 0.05)]
        pitch = np.zeros(4)
        mujoco.mju_axisAngle2Quat(pitch, [1, 0, 0], math.radians(rng.uniform(-6, 6)))
        quat = np.zeros(4)
        mujoco.mju_mulQuat(quat, self.cam_quat0, pitch)
        model.cam_quat[self.camera] = quat

    def _sensor_effects(self, image):
        rng = self.rng
        image = image.astype(np.float32) * rng.uniform(0.7, 1.3)  # exposure
        image += rng.normal(0, rng.uniform(1, 6), image.shape)    # sensor noise
        image = np.clip(image, 0, 255).astype(np.uint8)
        if rng.random() < 0.3:  # motion blur while turning
            k = int(rng.integers(3, 9))
            kernel = np.zeros((k, k), np.float32)
            kernel[k // 2] = 1.0 / k
            image = cv2.filter2D(image, -1, kernel)
        return image

    # ---------------------------------------------------------------- frame
    def frame(self):
        rx, ry, ryaw = self._robot_pose()
        self.sim.reset(rx, ry, ryaw)
        self._place_entities(rx, ry, ryaw)
        self._appearance()
        mujoco.mj_forward(self.sim.model, self.sim.data)

        self.renderer.disable_segmentation_rendering()
        self.renderer.update_scene(self.sim.data, camera='front_camera',
                                    scene_option=self.view)
        rgb = self.renderer.render()
        self.renderer.enable_segmentation_rendering()
        self.renderer.update_scene(self.sim.data, camera='front_camera',
                                    scene_option=self.view)
        seg = self.renderer.render()

        ids, types = seg[..., 0], seg[..., 1]
        pixels = {}
        geom_pixels = types == int(mujoco.mjtObj.mjOBJ_GEOM)
        for geom in np.unique(ids[geom_pixels]):
            key = self.geom_instance.get(int(geom))
            if key is not None:
                pixels.setdefault(key, []).append(geom_pixels & (ids == geom))
        labels = []
        for key, masks in pixels.items():
            mask = np.logical_or.reduce(masks)
            if mask.sum() < MIN_PIXELS:
                continue
            ys, xs = np.nonzero(mask)
            x1, x2, y1, y2 = xs.min(), xs.max() + 1, ys.min(), ys.max() + 1
            if min(x2 - x1, y2 - y1) < MIN_SIDE:
                continue
            model_name = 'cargo_box' if key.startswith('crate:') else self.body_model[key]
            cls = ENTITY_CLASS[model_name]
            labels.append((cls, (x1 + x2) / 2 / WIDTH, (y1 + y2) / 2 / HEIGHT,
                           (x2 - x1) / WIDTH, (y2 - y1) / HEIGHT))
        return self._sensor_effects(rgb), labels


def draw(image, labels):
    colors = [(255, 0, 255), (0, 165, 255), (0, 200, 0)]
    out = image.copy()
    for cls, cx, cy, w, h in labels:
        x1, y1 = int((cx - w / 2) * WIDTH), int((cy - h / 2) * HEIGHT)
        x2, y2 = int((cx + w / 2) * WIDTH), int((cy + h / 2) * HEIGHT)
        cv2.rectangle(out, (x1, y1), (x2, y2), colors[cls], 2)
        cv2.putText(out, CLASSES[cls], (x1, max(12, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, colors[cls], 1, cv2.LINE_AA)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--frames', type=int, default=1000)
    parser.add_argument('--out', type=Path, default=Path.home() / 'synthetic_dataset')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--min-objects', type=int, default=1,
                        help='re-render frames with fewer labelled objects')
    parser.add_argument('--preview', action='store_true',
                        help='also write images with the labels drawn on them')
    args = parser.parse_args()

    out = args.out.expanduser()
    (out / 'images').mkdir(parents=True, exist_ok=True)
    (out / 'labels').mkdir(parents=True, exist_ok=True)
    # Beside, not inside, the dataset: an uploaded folder must hold only
    # labelled images.
    preview = out.parent / f'{out.name}_preview'
    if args.preview:
        preview.mkdir(exist_ok=True)

    generator = Generator(args.seed)
    counts = np.zeros(len(CLASSES), int)
    written = 0
    while written < args.frames:
        image, labels = generator.frame()
        if len(labels) < args.min_objects:
            continue
        stem = f'sim_{args.seed:03d}_{written:06d}'
        bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(out / 'images' / f'{stem}.jpg'), bgr,
                    [cv2.IMWRITE_JPEG_QUALITY, int(generator.rng.integers(70, 96))])
        (out / 'labels' / f'{stem}.txt').write_text(''.join(
            f'{c} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n' for c, cx, cy, w, h in labels))
        if args.preview:
            cv2.imwrite(str(preview / f'{stem}.jpg'), draw(bgr, labels))
        for label in labels:
            counts[label[0]] += 1
        written += 1
        if written % 100 == 0:
            print(f'{written}/{args.frames} frames', flush=True)

    (out / 'data.yaml').write_text(
        f'nc: {len(CLASSES)}\nnames: {CLASSES}\n')
    print(f'Wrote {written} frames to {out}; objects per class: '
          + ', '.join(f'{n}={c}' for n, c in zip(CLASSES, counts)))


if __name__ == '__main__':
    main()
