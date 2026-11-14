# dot-sim

**Native twins of hot script code**, one optional GDExtension (Rust, godot-rust `godot` 0.5.5), one module per job: bots (dot-bot's sight scan), snapshot (dot-net's snapshot encode and decode), interp (dot-net's per-frame interpolator draw), motor (dot-player-controller's collision queries and whole motor tick, `DotSimFpsBody`). The script original stays the specification and the fallback.

Read the family [CLAUDE.md](../../CLAUDE.md) first; this file is only what is specific to dot-sim. The reasoning is in [docs/design.md](docs/design.md), the browser build and its status in [docs/web.md](docs/web.md).

## Rules that bite

- **Bit-identical, not close.** A native kernel gives the script's exact answer: f32 vector math in the engine's order, f64 where the script computes in floats, one rounding to f32 where the script stores into a packed float array. The suite compares to the bit. [design](docs/design.md#precision)
- **Nothing may need dot-sim.** Callers reach a native class only through `ClassDB` by name (`DotSim.has`, `ClassDB.instantiate`), never as a typed class, so scripts compile and run without the library.
- **Never commit a built library or the `.gdextension`.** Both are gitignored and written by `ci/install.sh`; a checkout without them runs the script path, which is what the web client always does.
- **Plain web exports never carry the extension.** With the plain template, the `.gdextension` alone stops the game's scripts. Only a dlink nothreads export may carry it (the `web-wasm32` release zip, built by `ci/web.sh`). [web](docs/web.md)
- **Pure crates hold the logic.** Engine types only in `dot-sim-gdext`; kernels take plain arrays so `cargo test` covers them.
- **The snapshot wire format is frozen by the script codec.** Native encode and decode produce the same bytes; golden-byte tests guard it.

## Running and testing

- `cargo clippy --manifest-path rust/Cargo.toml --locked -- -D warnings`, `cargo fmt --check` and `cargo test`; `ci/install.sh` builds and installs the host library; `ci/check.sh` checks the matrix ([native-ci.md](docs/native-ci.md)). On a shared build machine, through its build queue.
- Link `addons/dot_core` and `addons/dot_bot` from their repos, install a library, import once, then `timeout 300 godot --headless --path . res://examples/sim_selftest.tscn`.
- A change to a module's script original in another repo is not done until this suite passes with the library installed.
