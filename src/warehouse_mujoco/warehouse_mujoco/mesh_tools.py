"""Minimal binary-STL reader and vertex-clustering decimation."""

import numpy as np

MUJOCO_MAX_STL_FACES = 200000


def read_binary_stl(path):
    """Return an (N, 3, 3) float32 triangle array from a binary STL file."""
    raw = open(path, 'rb').read()
    count = int(np.frombuffer(raw, np.uint32, 1, 80)[0])
    record = np.dtype([('normal', '<f4', 3), ('v', '<f4', (3, 3)), ('attr', '<u2')])
    return np.frombuffer(raw, record, count, 84)['v']


def decimate(triangles, cell):
    """Merge vertices on a `cell`-sized grid and drop collapsed triangles."""
    vertices = triangles.reshape(-1, 3)
    keys = np.floor(vertices / cell).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    inverse = inverse.ravel()
    merged = np.zeros((inverse.max() + 1, 3))
    np.add.at(merged, inverse, vertices)
    merged /= np.bincount(inverse)[:, None]
    faces = inverse.reshape(-1, 3)
    keep = ((faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2])
            & (faces[:, 0] != faces[:, 2]))
    faces = faces[keep]
    _, first = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    faces = faces[np.sort(first)]
    return merged.astype(np.float32), faces.astype(np.int32)


def load_for_mujoco(path, max_faces=MUJOCO_MAX_STL_FACES // 2):
    """Return (vertices, faces) with at most max_faces, or None if not needed."""
    triangles = read_binary_stl(path)
    if len(triangles) <= MUJOCO_MAX_STL_FACES:
        return None
    cell = 0.001
    while True:
        vertices, faces = decimate(triangles, cell)
        if len(faces) <= max_faces:
            return vertices, faces
        cell *= 1.5
