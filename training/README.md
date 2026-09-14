# YOLO training workspace

Copy or link a YOLO-format dataset under `datasets/`, then adapt the examples
in `configs/`. Dataset contents and generated `runs/` are intentionally ignored
by Git; only reproducible configuration and scripts belong in the repository.

The training/evaluation/export wrappers expect the Ultralytics `yolo` command
to be available in the active Python environment.
