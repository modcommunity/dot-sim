#!/usr/bin/env bash
# Puts a built dot-sim library into addons/dot_sim/bin under its platform name and writes the
# .gdextension beside it, so the engine loads it. Both are gitignored: a checkout without a
# library (and every web export) runs the script fallback.
#   ci/install.sh <library file> <platform> <arch>     e.g. rust/target/release/libdot_sim.so linux x86_64
#   ci/install.sh --remove                              back to script only
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
bin="$here/addons/dot_sim/bin"
ext="$here/addons/dot_sim/dot_sim.gdextension"
if [[ "${1:-}" == "--remove" ]]; then
	rm -rf "$bin" "$ext" "$ext.uid"
	exit 0
fi
lib="$1" platform="$2" arch="$3"
case "$platform" in
	linux) name="libdot_sim.linux.$arch.so" ;;
	windows) name="dot_sim.windows.$arch.dll" ;;
	macos) name="libdot_sim.macos.$arch.dylib" ;;
	*) echo "unknown platform $platform" >&2; exit 2 ;;
esac
mkdir -p "$bin"
cp "$lib" "$bin/$name"
cp "$here/ci/dot_sim.gdextension.in" "$ext"
echo "installed $bin/$name; run an import so the engine lists the extension"
