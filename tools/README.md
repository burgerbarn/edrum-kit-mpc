# Development tools

- `replay.cpp` — feeds a logged MIDI stream (lines `STATUS D1 D2`) through the plugin natively under
  AddressSanitizer/UBSan and reports non-finite samples and peak. Build from the plugin folder inside the SDK:
  `clang++ -std=c++17 -O1 -g -fsanitize=address,undefined -I../../framework -Isrc -o replay tools/replay.cpp`
- `render.cpp` — offline render of a timestamped MIDI log (`unix_time STATUS D1 D2`) to a 24-bit WAV, with an optional
  file of corrected times. Set `EDRUM_KIT_DIRS` (kit folder) and `MPC_PLUGIN_FOLDER` (a folder with `plugin.json` and
  `settings.txt`); `FIXED_GAIN_DB` sets a fixed output gain instead of peak-normalising.
- `align_to_video.py` — finds the offset between a render and a phone video's audio and snaps hits to the video's onsets.
- `drmgrab.c` — read-only grab of the framebuffer an MPC is showing (DRM planes), run on the MPC over developer-mode SSH;
  `fb_to_png.py` turns its output into a PNG (rotated to the landscape screen).
- `mpctouch.c` — taps and swipes on the MPC touchscreen (evdev), for driving the UI remotely during testing.

`drmgrab` / `mpctouch` are static aarch64 binaries: `zig cc -target aarch64-linux-musl -O2 -static -o mpctouch mpctouch.c`.
They only run with the MPC's developer mode (SSH) switched on; use with care, they act on the live device.
