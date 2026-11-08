# dot-sim design

## What it is

Native (Rust, GDExtension) twins of hot script code in the dot family, one module per job. Each module has a script original that stays the specification and the fallback: the game chooses at run time with `ClassDB.class_exists`, and the suite holds the two to the same answers. Nothing in a game may need dot-sim to run.

| Module | Crate | Native class | Script original | State |
| --- | --- | --- | --- | --- |
| bots | `dot-sim-bots` | `DotSimBotSenses` | `DotBotBrain._candidates` (dot-bot) | the sight scan |
| snapshot | `dot-sim-snapshot` | `DotSimSnapshotCodec` | `DotNetVar.write`/`read` behind `DotNetBehaviour._encode`/`read_state` (dot-net) | encode and decode; the same bytes, held by golden-byte and parity tests |
| motor | `dot-sim-motor` | `DotSimFpsBody` | `DotFpsPhysicsBody` (dot-player-controller): sweep, rest contact, overlap and their fallbacks | the motor's collision queries, to the bit (dot-player-controller's `motor_native_parity` replays recorded inputs at 22/33/66 Hz and fuzzes the queries); the motor's slide loop stays in script |

## Layout

| Path | What |
| --- | --- |
| `rust/crates/dot-sim-math` | `Vec3` and `rad_to_deg`, rounded as the engine rounds (f32 vectors, f64 script floats) |
| `rust/crates/dot-sim-bots` | pure kernels over plain arrays; no engine types, tested with `cargo test` |
| `rust/crates/dot-sim-snapshot` | the snapshot codec, pure, like bots |
| `rust/crates/dot-sim-motor` | the motor's collision body over a `Space` trait (the engine queries), pure and tested with a fake floor; the gdext side is the engine `Space` |
| `rust/crates/dot-sim-gdext` | the one cdylib (`dot_sim`): thin bindings, one cargo feature per module (`bots`, `snapshot`, `motor`), `web`/`web-nothreads` for the browser build |
| `addons/dot_sim/dot_sim.gd` | `DotSim.has(module)`: which modules are loaded, safe with no library |
| `ci/` | build matrix, installer and packaging ([native-ci.md](native-ci.md)); the installer writes the gitignored `addons/dot_sim/bin/<platform>/` and `.gdextension` |
| `examples/sim_selftest.tscn` | the bots parity suite |
| `examples/snapshot_selftest.tscn` | the snapshot parity suite (needs dot-net linked at `addons/dot_net`) |

Kernels live in pure crates so they test without an engine and a second binding (a standalone server, a tool) costs nothing.

## Precision

The native code must give the bit-identical answer, not a close one, or a bot's choices drift from the script's over a match. The rules:

- `Vector3` is f32; its `length` is `sqrt` of `x*x + y*y + z*z` in f32, its `angle_to` is `atan2(|a x b|, a . b)` in f32.
- Script floats are f64: `rad_to_deg` multiplies by `180 / PI` in f64, and score arithmetic happens in f64 then rounds once to f32 when stored in a `PackedFloat32Array`.
- No fused multiply-add and no reordering: the Rust spells each expression in the engine's order.

## The bots boundary

Measured first (cp-orange-x3, 31 bots, headless on the build machine, per frame at 50 fps): the bot brain is 1.6 ms of a 5.7 ms game tick; inside one think call, perception is 27 µs of which the sight scan (`_candidates`) is 14 µs, hearing 14 µs, the fight 18 µs. The brain is no longer the dominant cost, so the native ceiling for bots is about 1.3 ms per tick, and the first kernel is the one pure, allocation-heavy loop: the sight scan.

- The game bumps `DotBotWorld.contacts_version` once per tick (game-arena sets it to the tick number).
- The first bot to think in a tick packs the bodies once into `DotSimBotSenses` (ids, aim points, teams, alive; struct of arrays).
- Each bot's scan is one call returning candidate indices; the scores and the "visible target died" flag are read after. The brain maps indices back to its `DotBotContact` list, so everything after the scan is unchanged script.
- `DotBotWorld.native_enabled` (static) switches it off for the suite and for comparisons; a world that never bumps the version always uses the script.
- dot-bot names the class only as a `StringName` for `ClassDB`, so dot-bot does not depend on dot-sim.
- Only the main thread scans natively. The packed senses keep the last scan's answer, so a brain thinking on a worker thread (game-arena's `bot_threads`) scans in script.
- Games expose the switch as a setting (game-arena: cvar `bot_native`, on by default); without the library it is script either way.

## Measured (build machine, headless, script fast scan against native)

| Run | Script | Native |
| --- | --- | --- |
| Scan over 32 bodies (suite) | 17.3 µs | 2.8 µs |
| game-arena server, 32 bots, 66 Hz: script ms/tick (bot brain) | 9.76 (2.20) | 9.30 (1.90) |
| game-arena server, 128 bots, 66 Hz: script ms/tick (bot brain) | 43.7 (13.2) | 39.4 (8.4) |
| game-arena offline, cp-orange-x3, 31 bots: game tick ms, two rounds | 6.02, 6.67 | 5.48, 5.63 |

The saving grows with the bot count because the scan is quadratic in bodies. At 128 bots the brain is no longer the main cost of the tick, so the next native candidates are outside the sight scan.

## The snapshot boundary

`DotSimSnapshotCodec` is one behaviour's declarations (kinds, bits, ranges, pads) and two calls: `encode(values, changed, pad)` writes a state exactly as `DotNetBehaviour._encode` does, and `decode(bytes, pad)` reads one exactly as `read_state` does. dot-net drives it through `DotNetSimCodec`; dot-net names the class only as a `StringName` for `ClassDB`, so it does not depend on dot-sim.

- Any value the script writes differently (a custom codec, a string, a 40-bit float range, a value of the wrong type) makes that state's encode fall back to the script; on decode the codec stops with `SCRIPT`, the script reads that one value, and the native read continues.
- Only the main thread encodes natively; snapshot workers keep the script.
- Off with `DotNetConfig.sim_codec` (`DOT_NET_SIM_CODEC=0`, `--net-sim-codec=false`) or `DotNetSimCodec.enabled`; the web never loads it.
- dot-net's own C++ core (`DotNetConfig.native`) already encodes on the server, so the codec matters there only with that core off; on a client it is the decode.

`examples/snapshot_selftest.tscn` holds it: 4000 random states encoded by both at every pad, every packet plus cut-short and noise packets decoded by both, and five scripted three-peer games (22 and 33 Hz, 10-20 % seeded loss, duplicates and jitter, interest rings, workers) whose payloads and received values must hash the same. The crate's `golden_player_state` test pins the bytes of one arena-like state.

## Measured: snapshot (build machine, headless)

| Run | Script | Native codec |
| --- | --- | --- |
| 128 player states, encode (suite bench) | 9.91 ms | 2.81 ms |
| 128 player states, decode and apply (suite bench) | 11.15 ms | 3.22 ms |
| dot-net `snapshot_bench`, server send ms per tick per client, 32/64/128 peers, dot-net core off | 0.76 / 1.04 / 1.62 | 0.63 / 0.97 / 1.60 |
| the same with dot-net's native core on | 0.17 / 0.27 / 0.41 | 0.17 / 0.25 / 0.42 |

With dot-net's core off, a server's send is mostly per-pair bookkeeping, not encoding; with it on, the codec is not on the server's path. The win is the client's decode, about 3.5 times faster.

## Shipping

The libraries are built per platform (linux, windows, macos; x86_64 and arm64) by CI and shipped as release-asset zips, never committed ([native-ci.md](native-ci.md)). A dedicated server unpacks the zip for its platform at the project root (or builds with `ci/install.sh`) and imports once so the engine lists the extension. Web exports never carry them ([web.md](web.md)), so the web build and its size gate are untouched.
