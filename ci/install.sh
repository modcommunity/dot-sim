#!/bin/bash
# Build the native library for one platform and install it where the addon loads it.
#   ci/install.sh [--platform P] [--profile release|debug] [--no-build] [--gdextension-only]
# P is a row of ci/targets.tsv (default: this machine's). Writes
#   addons/dot_sim/bin/<platform>/<library file>
#   addons/dot_sim/dot_sim.gdextension      (every platform whose library is present)
# Both are gitignored: the build is an artifact, never a commit. Without the library the
# addon runs its GDScript twins, so a missing or failed build is never fatal to a game.
# --no-build installs a library already built under rust/target. Cross builds need the rust
# target installed (rustup target add); the workflows do that. Exit 3: no rust/ crate yet.
# --gdextension-only rewrites the .gdextension from what is installed and builds nothing (ci/web.sh).
# The cargo step is the only heavy one: run this under `heavy` (or on a box with cargo).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
PLATFORM=""; PROFILE=release; BUILD=1; INSTALL=1
while [ $# -gt 0 ]; do
    case $1 in
        --platform) PLATFORM="${2:?}"; shift 2 ;;
        --profile) PROFILE="${2:?}"; shift 2 ;;
        --no-build) BUILD=0; shift ;;
        --gdextension-only) BUILD=0; INSTALL=0; shift ;;
        *) echo "install.sh: unknown argument '$1'" >&2; exit 2 ;;
    esac
done
case $PROFILE in release|debug) ;; *) echo "install.sh: profile must be release or debug" >&2; exit 2 ;; esac

host_platform() {
    local os arch
    os="$(uname -s)"; arch="$(uname -m)"
    case "$os/$arch" in
        Linux/x86_64) echo linux-x86_64 ;;
        Linux/aarch64|Linux/arm64) echo linux-arm64 ;;
        Darwin/arm64) echo macos-arm64 ;;
        Darwin/x86_64) echo macos-x86_64 ;;
        MINGW*/*|MSYS*/*|CYGWIN*/*) echo windows-x86_64 ;;
        *) echo "install.sh: no platform for $os/$arch" >&2; return 1 ;;
    esac
}

# The library's file stem: [lib] name if set, else the package name, dashes to underscores.
# In a workspace (rust/Cargo.toml has [workspace]) it is read from the crate building the cdylib.
crate_name() {
    local n m=rust/Cargo.toml
    if grep -q '^\[workspace\]' "$m"; then
        m="$(grep -l 'cdylib' rust/crates/*/Cargo.toml | head -n 1)"
        [ -n "$m" ] || { echo "install.sh: no cdylib crate under rust/crates" >&2; return 1; }
    fi
    n="$(awk '/^\[/{s=$0} s=="[lib]" && $1=="name"{gsub(/[" ]/,"",$3); print $3; exit}' "$m")"
    [ -n "$n" ] || n="$(awk '/^\[/{s=$0} s=="[package]" && $1=="name"{gsub(/[" ]/,"",$3); print $3; exit}' "$m")"
    [ -n "$n" ] || { echo "install.sh: no crate name in rust/Cargo.toml" >&2; return 1; }
    echo "${n//-/_}"
}

[ -f rust/Cargo.toml ] || { echo "install.sh: rust/Cargo.toml not found; nothing to build (the addon runs on GDScript)" >&2; exit 3; }
CRATE="$(crate_name)"
if [ "$INSTALL" = 1 ]; then
    [ -n "$PLATFORM" ] || PLATFORM="$(host_platform)"
    R="$(awk -F'\t' -v p="$PLATFORM" '$0!~/^#/ && $1==p {print; exit}' ci/targets.tsv)"
    [ -n "$R" ] || { echo "install.sh: '$PLATFORM' is not in ci/targets.tsv" >&2; exit 2; }
    IFS=$'\t' read -r _ TARGET _ PATTERN _ <<<"$R"
    FILE="${PATTERN//\{crate\}/$CRATE}"

    if [ "$BUILD" = 1 ]; then
        flags=(--manifest-path rust/Cargo.toml --target "$TARGET" --locked)
        [ "$PROFILE" = release ] && flags+=(--release)
        cargo build "${flags[@]}"
    fi
    SRC="rust/target/$TARGET/$PROFILE/$FILE"
    [ -f "$SRC" ] || { echo "install.sh: $SRC not found (build it, or drop --no-build)" >&2; exit 1; }
    DEST="addons/dot_sim/bin/$PLATFORM"
    mkdir -p "$DEST"
    cp "$SRC" "$DEST/$FILE"
fi

# Regenerate the .gdextension from what is installed, so a partial install is still valid.
OUT=addons/dot_sim/dot_sim.gdextension
{
    echo "[configuration]"
    echo "entry_symbol = \"gdext_rust_init\""
    echo "compatibility_minimum = 4.1"
    echo "reloadable = false"
    echo
    echo "[libraries]"
    awk -F'\t' '$0!~/^#/ && NF>=5 {print $1"\t"$4"\t"$5}' ci/targets.tsv | while IFS=$'\t' read -r plat pat key; do
        f="${pat//\{crate\}/$CRATE}"
        if [ -f "addons/dot_sim/bin/$plat/$f" ]; then echo "$key = \"res://addons/dot_sim/bin/$plat/$f\""; fi
    done
    # The web side module (ci/web.sh, opt-in): only a dlink web export may carry this file (docs/web.md).
    if [ -f "addons/dot_sim/bin/web-wasm32/$CRATE.wasm" ]; then echo "web.wasm32 = \"res://addons/dot_sim/bin/web-wasm32/$CRATE.wasm\""; fi
} > "$OUT"
[ "$INSTALL" = 1 ] && echo "installed $DEST/$FILE" || echo "wrote $OUT"
