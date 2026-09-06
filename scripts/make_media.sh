#!/usr/bin/env bash
# Regenerate the README images and recording. Needs the visualizer built:
#   cmake -S . -B build -DARM_BUILD_VIZ=ON && cmake --build build
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
viz="$root/build/viz/arm_viz"
bench="$root/build/arm_bench"
docs="$root/docs"
frames="$(mktemp -d)"
trap 'rm -rf "$frames"' EXIT

mkdir -p "$docs"

"$viz" --screenshot "$docs/workspace.png" --width 1280 --height 800
"$viz" --screenshot "$docs/dual_interpolation.png" --demo 0.75 --width 1280 --height 800
"$viz" --screenshot "$docs/joint_space.png" --view joint --dof 2 --width 1280 --height 800
"$viz" --screenshot "$docs/supervisor.png" --supervisor --width 1040 --height 660

# The recording: the same seeded command stream driving a supervised arm and an
# unsupervised one, at the controller's own 50 Hz.
"$viz" --frames "$frames" --frame-count 300 --frame-dt 0.02 --supervisor \
       --width 1040 --height 660 >/dev/null
python3 "$root/scripts/make_gif.py" "$frames" "$docs/supervisor.gif" \
        --scale 0.75 --every 2 --delay-ms 80 --colors 64

# The table that goes with it.
"$bench" --commands 2000 --log-dir "$root/bench_logs" --markdown "$root/bench/results.md"
