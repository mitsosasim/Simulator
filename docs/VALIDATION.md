# Documentation validation

Updated on 18 October 2026 for the thesis portfolio branch. Application source, runtime launch files, dependency manifests, and experiments remain unchanged.

## Supplied academic documents

| Check | Outcome |
|---|---|
| Thesis import | The supplied 67-page PDF is stored unchanged as `docs/thesis/report.pdf`, with byte equality verified against the upload |
| Slide archive import | The supplied ZIP contains exactly 25 numbered 1920 × 1080 JPEG slides, preserved unchanged as `presentation-slides.zip` |
| Presentation assembly | The PDF contains 25 pages in numerical order, each covering a 16:9 page with its original JPEG embedded without recompression |
| Presentation integrity | Every PDF image was extracted and compared byte-for-byte against its corresponding ZIP entry |
| Reproducibility | The assembly script was run from a separate directory with a fresh copy of the source archive, then its PDF compared with the committed build |
| Visual inspection | All 67 report pages and all 25 presentation pages were rendered and inspected, with selected figures viewed at full resolution |
| Report preservation | Original wording, figures, formatting, and existing editorial issues are retained. The report was not recompiled or rewritten |
| Sources | The supplied ZIP contains slide images, with no editable LaTeX or PowerPoint sources. No Overleaf build is claimed |

## Portfolio presentation

| Check | Outcome |
|---|---|
| Academic facts | Title, author, university, supervisor, date, and acknowledgments checked against the report |
| Results | Chapter 9 metrics are documented separately from the existing CSV. The positional improvement and slightly worse yaw RMSE in the thesis run are both described |
| Implementation context | Current C++ comparison filter, class labels, idealized IMU, fixed landmark map, and available dataset checked against source |
| README visual sources | The workspace screenshot matches slide 20 exactly. Report figures match their original PDF image objects |
| Links and images | Authored Markdown navigation, local files, section anchors, PDF page references, and image paths checked |
| README layout | Local Chromium rendering inspected at 1280 px and 390 px widths, with images loaded and no page-level horizontal overflow |
| Repository diff | Whitespace and scope checked, with no application source changes |

The browser check uses local Markdown with GitHub-like styling. It does not claim to reproduce GitHub's live renderer exactly. Full-resolution image and PDF links are available for detailed inspection.

## Previously verified development path

The existing README example executed and read position RMSE values of 0.320 m and 0.503 m from the tracked CSV. Authored Bash examples passed syntax checks. During the preceding portfolio update, the exact documented Docker command completed Catkin builds of `utils_ros` and `traffic_light_pkg`, then imported the generated `IMU` and `SignLabelArray` messages. Those application checks were not repeated for this document-only update.

The moved simulation recording and historical TF graph/PDF were previously verified byte-identical to their root-level originals.

## Remaining limits

The final `demo.mp4` is intentionally left for the author to upload. [Instructions and the reserved location](media/README.md#upload-the-final-demo) remain available.

Full simulator, perception, and historical rosbag evaluation were not rerun. The current cloud setup lacks the full ROS/Gazebo dependencies following the package-server network restriction described in the [setup guide](SETUP.md). Trained weights, original experiment rosbags, and run manifests are absent. The documents describe historical experiments without claiming fresh runtime validation or physical vehicle deployment.
