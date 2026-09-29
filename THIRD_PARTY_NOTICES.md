# Third-party notices

## OpenAMRobot simulation components

- Upstream: <https://github.com/openAMRobot/openamr-platform-sw>
- Local package: `src/openamrobot_description` (the imported `openamrobot_gazebo`
  package was removed when the simulation moved to MuJoCo)
- Upstream software license: MIT
- CAD-derived mesh license: CERN-OHL-P-2.0
- Imported on: 2026-09-18
- Local import commit: `13c76cce0b8c907e2c578fc281ccf37b7b01342b`
- Upstream revision: not recorded by the original import and therefore not asserted here

The integration keeps the mobile-base description and adds local launch and
warehouse configuration. Original copyright, attribution
and license terms remain with their respective upstream authors. The bundled
`LICENSE`, `LICENSING.md` and `UPSTREAM.md` files in
`src/openamrobot_description` contain the applicable terms
and provenance details.

`src/warehouse_mujoco/mjcf/amr.xml` is a MuJoCo translation of the upstream
MIT-licensed `robo_urdf.urdf.xacro` (link poses, masses, inertias and
collision primitives). It references the CERN-OHL-P-2.0 meshes from
`src/openamrobot_description` at run time and does not copy or ship them.
