# Third-party material

- `ros2_ws/src/tugger_nav/vda5050/order.schema`, `state.schema`, `instantActions.schema`: copied from
  https://github.com/VDA5050/VDA5050 (tag 2.1.0, commit 511d01d71587e8a3dc5e71d4f41dc52466f4284c, folder `json_schemas/`). Only change: one en dash
  in a description string of state.schema replaced with "-" (no effect on validation).
  MIT licence, copyright 2024 Verband der Automobilindustrie; the licence text is in `ros2_ws/src/tugger_nav/vda5050/LICENSE.txt`.
- `ros2_ws/src/tugger_nav/behavior_trees/*.xml`: adapted from the default behaviour trees of Nav2
  (https://github.com/ros-navigation/navigation2, Apache-2.0).
- Build-time dependencies (not vendored): ROS 2 Jazzy, Nav2, nlohmann/json and pboettch/json-schema-validator
  (via `ros-jazzy-nlohmann-json-schema-validator-vendor`).
