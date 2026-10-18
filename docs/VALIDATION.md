# Documentation validation

Validated on 18 October 2026 for the thesis portfolio branch. Application source, runtime launch files, dependency manifests, and experiments were left unchanged.

| Check | Outcome |
|---|---|
| Source and Git history review | README contribution claims trace to the detector, EKF, wheel plugin, characterization, and evaluation code |
| Archived results | README example executed and printed position RMSE values of 0.320 m and 0.503 m from the existing CSV |
| Shell examples | All authored Bash blocks in the README, setup guide, and architecture notes passed `bash -n` |
| Software-only setup | Exact documented Docker command completed Catkin builds of `utils_ros` and `traffic_light_pkg`, then imported generated `IMU` and `SignLabelArray` messages |
| Relative links and images | Authored Markdown navigation, local files, section anchors, and image paths checked |
| README layout | Chromium inspected at 1280 px and 390 px widths, with all three images loaded and no page-level horizontal overflow |
| Architecture | Graphviz SVG regenerated from its editable DOT source and visually inspected against the active processing path |
| Asset organization | Moved MP4, TF graph, and TF PDF verified byte-identical to their original Git versions |
| Repository diff | Whitespace checked and changes reviewed for unintended source edits |

The browser check uses a local Markdown rendering with GitHub-like styling, rather than claiming to reproduce GitHub's live renderer exactly. Dense figures can be opened at full size.

## Outstanding material and runtime checks

The combined uploaded archive exceeded the file tool's 32 MiB transfer limit. The original thesis PDF, presentation, LaTeX sources, and additional figures could not be imported, read, compiled, or inspected. No substitute report, placeholder PDF, or fabricated Overleaf package was created. Import and validation of those original materials require accessible smaller attachments.

The final `demo.mp4` is intentionally left for the author to upload. [Instructions and the reserved location](media/README.md#upload-the-final-demo) are included.

Full simulator, perception, and historical rosbag evaluation were not rerun. The current cloud setup lacks the full ROS/Gazebo dependencies because access to the ROS snapshot package server is denied by network policy. Trained weights and the original experiment rosbag are absent. The setup guide identifies these boundaries and the readiness checks still required.

The old TF PDF was preserved without modification. It is a historical transform snapshot, not the requested thesis report.
