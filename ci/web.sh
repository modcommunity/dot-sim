#!/bin/bash
# Build the web side module (docs/web.md) and install it where a dlink web export loads it.
#   ci/web.sh [--no-build]        then: ci/package.sh --version X --platform web-wasm32 (the release asset)
# Writes addons/dot_sim/bin/web-wasm32/dot_sim.wasm and regenerates addons/dot_sim/dot_sim.gdextension
# with a `web.wasm32` line (both gitignored). A project holding that file must be exported with the
# single-threaded dlink template (Extensions Support on, Thread Support off): the plain web template
# refuses to start with any .gdextension in the pack.
# Needs: emsdk activated (emcc on PATH; 3.1.74 is the tested one), the pinned nightly with rust-src and
# the wasm32-unknown-emscripten target (NIGHTLY, default below), and GODOT4_BIN (godot-rust's
# api-custom dumps the API from the engine). Run under `heavy`: -Zbuild-std rebuilds std.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
# Newer nightlies reject -Z emscripten-wasm-eh=false (docs/web.md); move only after a browser try.
NIGHTLY="${NIGHTLY:-nightly-2026-01-10}"
TARGET=wasm32-unknown-emscripten
BUILD=1
while [ $# -gt 0 ]; do
    case $1 in
        --no-build) BUILD=0; shift ;;
        *) echo "web.sh: unknown argument '$1'" >&2; exit 2 ;;
    esac
done
[ -f rust/Cargo.toml ] || { echo "web.sh: rust/Cargo.toml not found" >&2; exit 3; }

if [ "$BUILD" = 1 ]; then
    command -v emcc >/dev/null || { echo "web.sh: emcc not on PATH (source emsdk_env.sh)" >&2; exit 2; }
    [ -n "${GODOT4_BIN:-}" ] || { echo "web.sh: set GODOT4_BIN to a Godot 4.7 binary" >&2; exit 2; }
    rustup toolchain list | grep -q "^$NIGHTLY" ||
        { echo "web.sh: first run: rustup toolchain install $NIGHTLY -c rust-src -t $TARGET --profile minimal" >&2; exit 2; }
    # From rust/, not --manifest-path: cargo reads rust/.cargo/config.toml (the side-module flags)
    # only from the working directory, and without them the link fails on `undefined symbol: main`.
    (cd rust && cargo "+$NIGHTLY" build --locked --release -p dot-sim-gdext --features web-nothreads \
        -Zbuild-std --target "$TARGET")
fi
# The web-wasm32 row of ci/targets.tsv: install.sh copies rust/target/<target>/release/dot_sim.wasm into
# addons/dot_sim/bin/web-wasm32/ and lists it (web.wasm32) in the .gdextension with whatever else is installed.
bash ci/install.sh --platform web-wasm32 --no-build
