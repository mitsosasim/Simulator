# Recordings and final demo

[Back to the project](../../README.md)

## Existing simulation recording

[Watch or download the archived simulation recording](simulation-signs.mp4).

This is the repository's existing `sim_signs.mp4`, moved here without changing its bytes. It contains a 640 × 480 simulation camera recording lasting approximately 4 minutes 51 seconds. It is not labeled as the final presentation demo.

The retained [camera-only image](../images/simulation-camera.png) is a frame extracted at 80 seconds from that recording. It depicts the simulated track and signs, with no added detection overlays.

The [final presentation PDF](../thesis/presentation.pdf#page=20) includes a separate screenshot of the complete simulation workspace.

## Upload the final demo

**Reserved upload location: `docs/media/demo.mp4`.** No empty video file or broken player is included.

1. Upload your final `demo.mp4` at that path, keeping the existing simulation recording as a separate artifact.
2. Replace the README's `DEMO_UPLOAD_SLOT` block with the snippet below. The presentation screenshot acts as a clickable poster linked to the real video. Replace the poster with a frame from the final demo if it shows a different scene.

   ```markdown
   [![Watch the final thesis demo](docs/images/thesis-workspace.jpg)](docs/media/demo.mp4)

   [Watch or download the final demo](docs/media/demo.mp4).
   ```

3. For GitHub's native inline video player, attach the MP4 in a GitHub Markdown editor and paste the generated `https://github.com/user-attachments/assets/...` URL into the README. A repository MP4 link may open a download or file page depending on the viewer, and HTML `<video>` is not a reliable GitHub README player.

If the file exceeds GitHub's upload limits, host the video using a supported attachment or video service and use its real URL. Do not rename the existing recording to imply it is the new final demo.
