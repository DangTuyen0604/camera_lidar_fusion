# v1.0.0 release checklist

Do not check an item from inspection alone; attach the command output or runtime
artifact to the release notes.

- [ ] 12/12 packages build from a fresh clone.
- [ ] Unit, integration and system tests pass with no skips.
- [ ] KITTI projection and ONNX inference run.
- [ ] Fusion publishes valid XYZ and calibration drift is detected.
- [ ] Mapping/localization and warehouse headless smoke tests pass.
- [ ] M01–M04 complete with load/unload and docking below 0.08 m.
- [ ] Pallet replans, worker stops/resumes, box is avoided, collisions remain 0.
- [ ] Five live runs for all benchmark scenarios create CSV and seven PNGs.
- [ ] `runtime-cpu` and `training` images build; runtime smoke test passes.
- [ ] Gate 7.1 audit and Gate 7.2 fresh-clone validation pass.
- [ ] README, architecture/topic/frame tables, screenshots and demo video exist.
- [ ] Changelog, limitations, Apache-2.0 license and CI badge are current.
- [ ] GitHub Actions is green on the exact release commit.
- [ ] Release branch is pushed before annotated tag `v1.0.0`.
