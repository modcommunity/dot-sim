#!/bin/bash
# Pack what ci/install.sh laid out for ONE platform into dist/ for the release:
#   dist/dot_sim-<version>-<platform>.zip   addons/dot_sim/bin/<platform>/* + the .gdextension
#   dist/SHA256SUMS                          (appended)
#   dist/natives.json                        {version, files:{platform:{file,sha256}}} (merged)
# The zip unpacks at a project root, like the GDScript addon zip dot-ci attaches.
#   ci/package.sh --version 0.1.0 --platform linux-x86_64
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; cd "$ROOT"
VERSION=""; PLATFORM=""
while [ $# -gt 0 ]; do
    case $1 in
        --version) VERSION="${2:?}"; shift 2 ;;
        --platform) PLATFORM="${2:?}"; shift 2 ;;
        *) echo "package.sh: unknown argument '$1'" >&2; exit 2 ;;
    esac
done
VERSION="${VERSION#v}"
[[ "$VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+([-+][0-9A-Za-z.-]+)?$ ]] || { echo "package.sh: --version must be semver" >&2; exit 2; }
awk -F'\t' -v p="$PLATFORM" '$0!~/^#/ && $1==p {f=1} END{exit !f}' ci/targets.tsv || { echo "package.sh: '$PLATFORM' not in ci/targets.tsv" >&2; exit 2; }
DIR="addons/dot_sim/bin/$PLATFORM"
{ [ -d "$DIR" ] && [ -n "$(ls -A "$DIR")" ]; } || { echo "package.sh: $DIR is empty (run ci/install.sh first)" >&2; exit 1; }
mkdir -p dist
ZIP="dist/dot_sim-$VERSION-$PLATFORM.zip"
rm -f "$ZIP"
python3 -I - "$ZIP" "$DIR" addons/dot_sim/dot_sim.gdextension <<'PY'
import sys, os, zipfile
zip_path, d, ext = sys.argv[1:4]
def add(z, path, mode):
    zi = zipfile.ZipInfo(path, (1980, 1, 1, 0, 0, 0))
    zi.external_attr = mode << 16
    with open(path, "rb") as f:
        z.writestr(zi, f.read(), zipfile.ZIP_DEFLATED)
with zipfile.ZipFile(zip_path, "w") as z:
    for name in sorted(os.listdir(d)):
        add(z, os.path.join(d, name), 0o755)
    add(z, ext, 0o644)
PY
SUM="$(sha256sum "$ZIP" | cut -d' ' -f1)"
echo "$SUM  $(basename "$ZIP")" >> dist/SHA256SUMS
python3 -I - "$VERSION" "$PLATFORM" "$(basename "$ZIP")" "$SUM" <<'PY'
import sys, json, os
v, plat, f, s = sys.argv[1:5]
p = "dist/natives.json"
m = json.load(open(p)) if os.path.exists(p) else {"version": v, "files": {}}
if m["version"] != v:
    sys.exit("natives.json is for " + m["version"] + ", not " + v)
m["files"][plat] = {"file": f, "sha256": s}
with open(p, "w") as out:
    json.dump(m, out, indent=2, sort_keys=True)
PY
echo "packed $ZIP"
