#!/usr/bin/env bash
# Run a command inside the pupper-ros-dev container with ros2_ws mounted at /ws.
# Build artefacts live in a named volume so the host tree stays clean.
set -euo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"
docker run --rm $([ -t 0 ] && echo -it) \
  -v "$WS":/ws \
  -v pupper-ros-dev-build:/ws/build -v pupper-ros-dev-install:/ws/install -v pupper-ros-dev-log:/ws/log \
  -w /ws pupper-ros-dev \
  bash -lc "source /opt/ros/jazzy/setup.bash && [ -f install/setup.bash ] && source install/setup.bash; $*"
