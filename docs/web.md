# dot-sim on the web

**Verdict: the web client keeps the script path.** A GDExtension written with godot-rust does run in a Godot 4.7.2 web export, verified by a real try (headless Chromium printed the extension's answer). It costs a second engine build, a pinned nightly toolchain and about 1 MB more gzipped download, for code the browser client does not need: dot-sim's modules are server-side (bots) or have a script twin with the same bytes (snapshot), so a web client without the extension plays on the same servers as a native one.

## What was tried

| Piece | Version |
| --- | --- |
| Engine | 4.7.2-stable, official export templates (they ship `web_dlink_*` and `web_dlink_nothreads_*`; no source build needed) |
| Template emscripten | 4.0.20 (the engine's own console line) |
| emsdk for the side module | 3.1.74 |
| rustc | nightly-2026-01-10 (godot 0.5.5 needs 1.94 or newer) |
| godot crate | 0.5.5, features `api-custom`, `experimental-wasm`, `lazy-function-tables`, `experimental-wasm-nothreads` |

Build, from `rust/` with emsdk activated and `GODOT4_BIN` set (for `api-custom`):

```
cargo +nightly-2026-01-10 build --release -p dot-sim-gdext --features web-nothreads -Zbuild-std --target wasm32-unknown-emscripten
```

The flags are in `rust/.cargo/config.toml` under `[target.wasm32-unknown-emscripten]`: `-sSIDE_MODULE=2`, emscripten C++ exceptions off, hidden default visibility, no native libraries, `-Z emscripten-wasm-eh=false`.

The export ticks **Extensions Support** and uses the single-threaded dlink template.

## Results

| Export | Console |
| --- | --- |
| dlink template, extension present | `Build configuration: Emscripten 4.0.20, single-threaded, GDExtension support.`, the extension's class answered, the game continued |
| plain template, project without the `.gdextension` | `no GDExtension support`, `ClassDB.class_exists` false, the script path ran, the game continued |
| plain template, project WITH the `.gdextension` | `Error: GDExtension libraries are not supported by this engine version`, and the probe script never ran |
| native Linux headless | the extension's class answered |

| File | Raw bytes | gzip -9 bytes |
| --- | --- | --- |
| plain `index.wasm` | 39,514,754 | 10,084,297 |
| dlink `index.wasm` (main module) | 1,508,095 | 582,004 |
| dlink `index.side.wasm` (engine) | 44,078,870 | 10,486,172 |
| the probe extension `.wasm` | 360,308 | 127,630 |

## Rules this sets

- **A web export never carries `addons/dot_sim/dot_sim.gdextension` or `addons/dot_sim/bin/`.** With the plain template the extension file alone stops the game's scripts. Both paths are gitignored and written only by `ci/install.sh`, so a checkout or CI pack has neither unless something installs them on purpose; a game that installs them for its server build excludes them in its web preset.
- Every native class is reached through `ClassDB.class_exists` (see `DotSim.has`), never by name in a typed script, so the scripts compile where it is absent.
- The newest nightly breaks the build: it rejects `-Z emscripten-wasm-eh=false`, and without the flag the page fails with `LinkError: ... "__cpp_exception": tag import requires a WebAssembly.Tag`. Pin a nightly that still takes the flag.

## Not tested

Firefox, Safari, the threaded templates, and a real game (only a probe project). A web build of dot-sim itself is not in CI.
