# Native build, CI and release

**For**: whoever changes the Rust crate or cuts a dot-sim release. The library is optional: every native function has a GDScript twin, and web clients only ever run the twins.

## Layout

| Path | What |
| --- | --- |
| `rust/` | the Cargo workspace (`rust/Cargo.toml`); `crates/dot-sim-gdext` builds the one `cdylib` (`dot_sim`), the other crates are plain Rust ([design](design.md)); every job skips with a notice while `rust/Cargo.toml` is absent |
| `ci/targets.tsv` | the platform matrix: platform, rust target, runner, library file, Godot feature key. **No web row, on purpose.** |
| `ci/install.sh` | build one platform and lay it out under `addons/dot_sim/bin/<platform>/`, then regenerate `addons/dot_sim/dot_sim.gdextension` (both gitignored) |
| `ci/package.sh` | zip one platform into `dist/dot_sim-<version>-<platform>.zip`, append `SHA256SUMS`, merge `natives.json` |
| `ci/check.sh` | offline: scripts parse, `targets.tsv` is whole, both workflows' matrices name every platform. No toolchain needed |

## Workflows

- `ci.yml`: dot-ci `validate` (GDScript), `ci/check.sh`, then a native job per platform: fmt and clippy (linux-x86_64 only), `cargo test`, `ci/install.sh`, the library uploaded as an artifact. Linux x86_64 and arm64, Windows x86_64, macOS arm64 and x86_64.
- `release.yml` (tag `v*`): validate, dot-ci's addon release (the GDScript zip), the native matrix, then `attach` uploads `dot_sim-<version>-<platform>.zip`, `natives.json` and `SHA256SUMS.natives` to the same GitHub release, verifying the sums first.

## How it reaches the CDN

It does not go from here. The site publishes an addon's pack from its release (hourly, or "Pull now") and holds the signing key, so this repository signs and uploads nothing to the content origin ([releases](../../dot-server-deploy/docs/releases.md)). A GDExtension must be a real file on disk, so the natives are never inside the mounted pack: the server install and the native client export unpack `dot_sim-<version>-<platform>.zip` at the project root, selecting by `natives.json`. The `.gdextension` lists only the platforms present, so a missing one means GDScript on that machine, never a load error.

## Local

```bash
bash ci/check.sh                               # no toolchain
heavy -- bash ci/install.sh                    # builds for this machine (cargo; the build is heavy)
bash ci/package.sh --version 0.1.0 --platform linux-x86_64
```

Build on a box with a Rust toolchain (the build machine has stable Rust; run cargo through its build queue).
