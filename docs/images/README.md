# Visual sources

The portfolio uses the author's supplied report and presentation as historical evidence. Figures were extracted or copied without adding detections, altering results, or synthesizing scenes.

| File | Original source | What it shows |
|---|---|---|
| [thesis-workspace.jpg](thesis-workspace.jpg) | Presentation archive `20.jpg`, unchanged | Gazebo track, sign-detection window, EKF console, and RViz |
| [thesis-vehicle.png](thesis-vehicle.png) | Report Figure 9, PDF page 29, printed page 22 | The simulated Ackermann car on the sign-rich track |
| [thesis-rviz.png](thesis-rviz.png) | Report Figure 14, PDF page 50, printed page 43 | The archived RViz trajectory comparison |
| [thesis-trajectories.png](thesis-trajectories.png) | Report Figure 15, PDF page 55, printed page 48 | Ground truth, BB EKF, and local comparison paths aligned to their starts |
| [simulation-camera.png](simulation-camera.png) | Frame at 80 seconds in the existing simulation recording | A camera-only view retained for the demo upload instructions |
| [architecture.svg](architecture.svg) | Export of [architecture.dot](architecture.dot) | Current implementation overview drawn from the source code |

Report figures were extracted directly from their PDF image objects. Their pixel dimensions and decoded pixels were checked against the supplied PDF. The full [report](../thesis/report.pdf) and [presentation](../thesis/presentation.pdf) preserve the original context and captions. Existing figure legends retain the historical `robot_localization` executable name, whose meaning is explained in the [architecture notes](../ARCHITECTURE.md#comparison-and-evaluation).

The architecture overview remains an editable explanation of the active code. Historical source diagrams inside the report and slides are preserved as supplied.
