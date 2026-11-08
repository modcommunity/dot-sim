#!/bin/bash
# Offline checks for the build matrix; needs no toolchain, so it runs anywhere (PH included).
#   ci/check.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; cd "$ROOT"
fail=0; bad() { echo "check: $*" >&2; fail=1; }
for f in ci/*.sh; do bash -n "$f" || bad "$f does not parse"; done
awk -F'\t' '$0!~/^#/ && NF!=5 {exit 1}' ci/targets.tsv || bad "targets.tsv: every row needs 5 tab-separated columns"
dups="$(awk -F'\t' '$0!~/^#/{print $1}' ci/targets.tsv | sort | uniq -d)"
[ -z "$dups" ] || bad "targets.tsv: duplicate platform $dups"
if grep -v '^#' ci/targets.tsv | grep -qi '^web'; then bad "targets.tsv: web must stay GDScript-only"; fi
for wf in .github/workflows/ci.yml .github/workflows/release.yml; do
    for p in $(awk -F'\t' '$0!~/^#/{print $1}' ci/targets.tsv); do
        grep -q "platform: $p\$" "$wf" || bad "$wf: matrix has no row for $p"
    done
done
exit $fail
