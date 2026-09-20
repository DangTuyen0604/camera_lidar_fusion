# Release demo runbook

Record one continuous run at 1080p or higher. Keep ROS timestamps and the
mission-state overlay visible. The required sequence is:

1. warehouse overview and active Nav2 nodes;
2. `/mission/start` accepted and cargo loaded;
3. live image, point cloud and fused XYZ visualization;
4. pallet insertion followed by a changed global path;
5. worker crossing, complete stop, worker clearing, motion resume;
6. fallen-box avoidance;
7. final docking inside the 0.08 m tolerance;
8. cargo unloaded and `COMPLETED`, followed by the next mission;
9. benchmark `summary.csv` and all seven plots.

Before recording, run `./scripts/run_audit.sh --full`. Start the visible demo
with `./scripts/run_release_demo.sh` and record the ROS topics listed
in `scripts/record_personal_bag.sh` for reproducibility. Video files are release
assets and must not be committed to Git.
