#!/usr/bin/env bash
# Builds, lints and tests the Rust workspace, then installs the host library (ci/install.sh).
#   ci/build.sh            release build of the host platform
#   ci/build.sh --check    clippy and tests only
# On a shared build machine, run it through the machine's build queue.
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
cd "$here/rust"
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
[[ "${1:-}" == "--check" ]] && exit 0
cargo build --release -p dot-sim-gdext
case "$(uname -s)" in
	Linux) platform=linux lib=target/release/libdot_sim.so ;;
	Darwin) platform=macos lib=target/release/libdot_sim.dylib ;;
	*) platform=windows lib=target/release/dot_sim.dll ;;
esac
arch="$(uname -m)"; [[ "$arch" == aarch64 ]] && arch=arm64
"$here/ci/install.sh" "$here/rust/$lib" "$platform" "$arch"
