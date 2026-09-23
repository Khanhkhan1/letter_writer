#!/usr/bin/env bash
# Apply macOS/RoboStack-only source patches this package's Gazebo stack
# needs, before building the workspace. No-op (and safe to run) on Linux --
# just skip this script there.
#
# Usage: run once after `vcs import` / cloning gz_ros2_control into
# ~/ros2_ws/src, before `colcon build`:
#   bash letter_writer/scripts/macos_robostack_patches.sh ~/ros2_ws/src
#
# What and why:
#
# 1. gz_ros2_control's plugin library ends up transitively linking some
#    rosidl_generator_py (Python extension) libraries, built with
#    "-undefined dynamic_lookup". On Linux this resolves fine at runtime;
#    on macOS it only resolves if libpython is itself a linked dependency
#    of the final .dylib. Without this, Gazebo fails to load the plugin
#    with:
#      dlopen(...libign_ros2_control-system.dylib): symbol not found in
#      flat namespace '_PyExc_RuntimeError'
#    and the simulated robot never gets a working controller_manager.
#    Fix: link Python3::Python into gz_ros2_control's plugin targets.
#
# 2. $CONDA_PREFIX/.../tinyxml2/tinyxml2-shared-targets.cmake exports the
#    target as tinyxml2::tinyxml2 (lowercase), but ignition-gui6's own
#    exported CMake config expects TINYXML2::TINYXML2 (uppercase) --
#    a version-skew packaging mismatch between the two conda-forge
#    packages. Fix: add an ALIAS under the name ignition-gui6 expects.
#    This one patches the active conda environment itself (not a source
#    tree), so it must be re-applied if the environment is recreated.

set -euo pipefail

SRC_DIR="${1:?Usage: $0 <path to ros2_ws/src>}"
GZ_ROS2_CONTROL_CMAKE="$SRC_DIR/gz_ros2_control/gz_ros2_control/CMakeLists.txt"

if [[ "$(uname)" != "Darwin" ]]; then
  echo "Not macOS, nothing to patch."
  exit 0
fi

# --- Patch 1: link libpython into gz_ros2_control's plugin targets ---
if [[ -f "$GZ_ROS2_CONTROL_CMAKE" ]]; then
  if grep -q "GZ_ROS2_CONTROL_PYTHON_LIB" "$GZ_ROS2_CONTROL_CMAKE"; then
    echo "gz_ros2_control CMakeLists.txt already patched."
  else
    python3 - "$GZ_ROS2_CONTROL_CMAKE" <<'PYEOF'
import re
import sys

path = sys.argv[1]
with open(path) as f:
    content = f.read()

anchor = "find_package(yaml_cpp_vendor REQUIRED)"
snippet = anchor + """

# RoboStack/macOS workaround: controller_manager's message dependencies
# transitively link rosidl_generator_py (Python extension) libraries built
# with "-undefined dynamic_lookup", which macOS can only resolve if
# libpython is an actual linked dependency of the final plugin .dylib.
set(GZ_ROS2_CONTROL_PYTHON_LIB)
if(APPLE)
  find_package(Python3 QUIET COMPONENTS Development)
  if(Python3_FOUND)
    set(GZ_ROS2_CONTROL_PYTHON_LIB Python3::Python)
  endif()
endif()"""
assert anchor in content, "anchor line not found, upstream file changed?"
content = content.replace(anchor, snippet, 1)

for target in ("${PROJECT_NAME}-system", "gz_hardware_plugins", "ign_ros2_control-system"):
    old = f"target_link_libraries({target}\n"
    if old not in content:
        continue
    idx = content.index(old) + len(old)
    close_idx = content.index(")", idx)
    if "GZ_ROS2_CONTROL_PYTHON_LIB" not in content[idx:close_idx]:
        content = content[:close_idx] + "\n  ${GZ_ROS2_CONTROL_PYTHON_LIB}" + content[close_idx:]

with open(path, "w") as f:
    f.write(content)
print("Patched", path)
PYEOF
  fi
else
  echo "WARNING: $GZ_ROS2_CONTROL_CMAKE not found -- clone/vcs-import gz_ros2_control first." >&2
fi

# --- Patch 2: tinyxml2 target alias in the active conda environment ---
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  TINYXML2_CMAKE="$CONDA_PREFIX/lib/cmake/tinyxml2/tinyxml2-shared-targets.cmake"
  if [[ -f "$TINYXML2_CMAKE" ]]; then
    if grep -q "TINYXML2::TINYXML2 ALIAS" "$TINYXML2_CMAKE"; then
      echo "tinyxml2 CMake config already patched."
    else
      sed -i.bak '/add_library(tinyxml2::tinyxml2 SHARED IMPORTED)/a\
if(NOT TARGET TINYXML2::TINYXML2)\
  add_library(TINYXML2::TINYXML2 ALIAS tinyxml2::tinyxml2)\
endif()
' "$TINYXML2_CMAKE"
      echo "Patched $TINYXML2_CMAKE"
    fi
  else
    echo "NOTE: $TINYXML2_CMAKE not found -- skipping tinyxml2 patch (package may not be installed yet)."
  fi
else
  echo "NOTE: \$CONDA_PREFIX not set -- activate the RoboStack conda env before running this script."
fi

echo "Done."
