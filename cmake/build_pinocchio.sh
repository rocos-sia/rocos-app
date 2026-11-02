#!/usr/bin/env bash
# 在指定 App 构建目录内安装固定版本，便于开发和 DEB 收集私有依赖。
set -euo pipefail
mkdir -p "${1:-build}"
build_dir=$(cd "${1:-build}" && pwd)
repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
pin_source="$repo_dir/3rdparty/pinocchio"
pin_commit=ed3bb75ce96cf26e84aebd8b73785407950a0f1f
if [[ ! -f "$pin_source/CMakeLists.txt" ]]; then
    echo "Initialize Pinocchio first: git submodule update --init 3rdparty/pinocchio" >&2
    exit 1
fi
if [[ "$(git -C "$pin_source" rev-parse HEAD)" != "$pin_commit" ]]; then
    echo "Pinocchio source must be at $pin_commit" >&2
    exit 1
fi
if [[ ! -f "$pin_source/cmake/base.cmake" ]]; then
    git -C "$pin_source" submodule update --init cmake
fi
cmake -S "$pin_source" -B "$build_dir/deps/pinocchio-build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$build_dir/deps/prefix" \
    -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
    -DBUILD_WITH_COLLISION_SUPPORT=OFF
cmake --build "$build_dir/deps/pinocchio-build" --parallel 2
cmake --install "$build_dir/deps/pinocchio-build"
mkdir -p "$build_dir/deps/prefix/share/licenses/pinocchio"
cp "$pin_source/LICENSE" "$build_dir/deps/prefix/share/licenses/pinocchio/LICENSE"
