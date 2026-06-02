# Documentation images

| File | Description |
|------|-------------|
| `architecture.svg` | espFoC component architecture (README) |
| `espfoc_tool_logo.svg` | espFoC Tool icon / README header |
| `espfoc_tool_logo.png` | Raster variant for window icon |
| `espfoc_demo.gif` | Hardware demo (motor running) |
| `espfoc_tool.gif` | *(optional)* espFoC Tool screen capture for README |

## Recording `espfoc_tool.gif`

When the UI is stable:

1. Run `axis_tuning` on hardware and connect espFoC Tool.
2. Capture 1280×720, ~15 s: connect → Tune → Dashboard (iq step, scope).
3. Export as GIF (e.g. `ffmpeg` or Peek) and save as `espfoc_tool.gif`.
4. Link from the root `README.md` tuning section.

Until then the SVG logo is used as the README visual.
