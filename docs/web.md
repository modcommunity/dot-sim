# dot-sim on the web

**Status: works, being shipped (decided 2026-11-13).** The real dot-sim extension, with all its modules, loads and runs in a Godot 4.7.2 web export built from the single-threaded dlink template. Its native paths are 3 to 7 times faster than the script twins in the browser, and their bytes equal the script's on wasm too. The script path stays as the complete fallback: every module has a script twin that produces the same bytes, and a web export without the side module runs it.

## Build

| Piece | Version |
| --- | --- |
| Engine | 4.7.2-stable, official export templates (they ship `web_dlink_*` and `web_dlink_nothreads_*`; no source build needed) |
| Template emscripten | 4.0.20 (the engine's own console line) |
| emsdk for the side module | 3.1.74 |
| rustc | nightly-2026-01-10 (godot 0.5.5 needs 1.94 or newer) |
| godot crate | 0.5.5, features `api-custom`, `experimental-wasm`, `lazy-function-tables`, `experimental-wasm-nothreads` |

`ci/web.sh` is the build. It is opt-in and needs emsdk activated, `GODOT4_BIN` set and the pinned nightly. It writes `addons/dot_sim/bin/web-wasm32/dot_sim.wasm` and a `.gdextension` with a `web.wasm32` line, and both are gitignored. It builds from `rust/` because cargo reads `rust/.cargo/config.toml` only from the working directory. Built with `--manifest-path` from the root, the side-module flags are lost and the link fails on `undefined symbol: main`. The config holds these flags under `[target.wasm32-unknown-emscripten]`:

- `-sSIDE_MODULE=2`
- emscripten C++ exceptions off
- hidden default visibility
- no native libraries
- `-Z emscripten-wasm-eh=false`

The export preset ticks **Extensions Support**, sets **Thread Support** off and uses the dlink nothreads template. Godot's release web templates take no scene path on the command line, so the tests below ran through a main scene that loads the scene named in `location.hash`.

## Measured (2026-11-13, PH, headless Chromium with SwiftShader; staging02 was down)

| Suite on the dlink export | Result |
| --- | --- |
| `sim_selftest` (bots, brain parity, sight) | 9 passed, 0 failed |
| `snapshot_selftest` (wire parity, codec) | 34 passed, 0 failed (33/1 before the `to_int` fix below) |

| Bench (same box, runs interleaved) | Script, plain export | Script, dlink export | Native, dlink export |
| --- | --- | --- | --- |
| sight scan, us | 56.5 / 60.3 / 62.0 | 61.2 / 63.2 / 65.6 | 8.0 (7.4x faster) |
| snapshot encode, 128 entities, ms | 13.04 / 13.33 | 11.26 / 10.71 | 2.95 / 2.91 (about 4x) |
| snapshot decode+apply, 128 entities, ms | 13.58 / 14.01 | 12.57 / 11.63 | 3.92 / 3.78 (about 3.3x) |

The dlink engine is built position-independent. On the sight scan this made the script path about 6% slower than on the plain engine. On the snapshot bench the dlink script runs were faster than the plain ones. So for the script path, the cost of the dlink engine is noise-sized either way.

| File | Raw bytes | gzip -9 bytes |
| --- | --- | --- |
| plain `index.wasm` | 39,514,754 | 10,084,297 |
| dlink `index.wasm` (main module) | 1,508,095 | 582,004 |
| dlink `index.side.wasm` (engine) | 44,078,870 | 10,486,172 |
| `dot_sim.wasm` (all modules) | 699,755 | 225,807 |

The dlink export with dot-sim is about 1.2 MB larger after gzip than the plain export (about +12%).

## Blockers before shipping

1. **dot-net gated the codec off on the web.** `DotNetSimCodec.available()` checked `not OS.has_feature("web")`; dot-net now checks only `ClassDB.class_exists` (the games link dot-net's checkout, so the one change reaches all of them).
2. **Encode parity on wasm: fixed.** GDScript's `int(float)` is the platform's conversion. On x86-64 (`cvttsd2si`) a NaN or out-of-range value is `i64::MIN`; under Emscripten (`trunc_sat`) a NaN is 0 and the rest saturates, measured in Chromium: `int(nan)` 0, `int(1e300)` `i64::MAX`, `int(-1e300)` `i64::MIN`. `to_int` (dot-sim-snapshot) hard-coded the x86 answer, so a NaN or 1e300 in an unsigned field encoded differently (13 of the probe's edge cases). It now follows the target: Rust's saturating `as` on wasm, the x86 emulation elsewhere. Native x86 output is unchanged.
3. **The FC web export has to change.** It needs the dlink nothreads template with Extensions Support, and the CDN must serve `index.side.wasm` and `dot_sim.wasm`. A plain template with any `.gdextension` in the pack refuses to start. That includes godot-sqlite's, which has web entries.
4. **The build box needs the pinned nightly** with `rust-src` and the emscripten target, plus emsdk 3.1.74. There is no web CI job yet.

## Rules

- **A plain-template web export never carries `addons/dot_sim/dot_sim.gdextension`.** With the plain template, that file alone stops the game's scripts (`GDExtension libraries are not supported by this engine version`). Only a dlink export may carry it, and only `ci/web.sh` writes the `web.wasm32` line.
- Every native class is reached through `ClassDB.class_exists` (see `DotSim.has`), never by name in a typed script, so the scripts compile where it is absent.
- The newest nightly breaks the build. It rejects `-Z emscripten-wasm-eh=false`, and without that flag the page fails with `LinkError: ... "__cpp_exception": tag import requires a WebAssembly.Tag`. Move `NIGHTLY` in `ci/web.sh` only after a browser try.
- `ci/targets.tsv` has no web row (`ci/check.sh` enforces that). The web build stays opt-in until the owner decides.

## Not tested

- Firefox and Safari
- the threaded templates
- a GPU browser (SwiftShader only)
- the motor (dot-player-controller) on the web
- a full FC client
