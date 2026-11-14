# dot-sim

Native twins of hot simulation code for the dot family of Godot addons, as one optional GDExtension written in Rust. Each module answers exactly as the script it replaces; a game that does not have the library, and every web export, runs the script.

| Module | Replaces |
| --- | --- |
| bots | the sight scan of dot-bot's brain |
| snapshot | (planned) the snapshot encoder and decoder, byte for byte |

## Use

1. `ci/install.sh` (needs a stable Rust toolchain) builds this machine's library into `addons/dot_sim/bin/<platform>/` and writes `addons/dot_sim/dot_sim.gdextension`; or unpack the release's `dot_sim-<version>-<platform>.zip` at the project root.
2. Open or import the project once so the engine lists the extension.

`DotSim.has(&"bots")` tells whether a module is loaded. dot-bot uses it on its own when present; nothing else is needed.

Keep `addons/dot_sim/bin/` and `dot_sim.gdextension` out of plain-template web exports; the opt-in dlink web build is `ci/web.sh`. See [docs/web.md](docs/web.md).

## Test

Link the sibling addons (`addons/dot_core`, `addons/dot_bot`), install a library, then:

```
godot --headless --path . res://examples/sim_selftest.tscn
```

Without a library the suite checks the fallback and says the native half was skipped. `cargo test` in `rust/` tests the kernels alone.

Design and numbers: [docs/design.md](docs/design.md).

## License

MIT
