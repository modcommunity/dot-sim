extends Node

## The interp module's suite: dot-net's [code]DotNetInterpolator.apply[/code] in GDScript and in
## [code]DotSimInterpolator[/code] on the same seeded streams (loss, reordering, duplicate and partial
## ticks, stalls, extrapolation past its cap, resumes eased and teleports not, vectors, rotations,
## colours, discrete and type-switching values, a write hook, a plan that changes, forget and prune),
## frame for frame. Every property written, every [code]_net_interpolated[/code] tick and every
## counter must be equal to the bit. With no library it checks the fallback and says the native half
## was skipped. With [code]DOT_SIM_REQUIRE_NATIVE=1[/code] (CI) a missing library fails instead.

const ENTITIES := 4
const FRAMES := 2400

## Every kind of value the interpolator draws, interpolated; one plain property it must not touch.
class Body extends DotNetBehaviour:
	var position: Variant = Vector3.ZERO
	var aim: Variant = Vector2.ZERO
	var yaw: Variant = 0.0
	var rot: Variant = Quaternion.IDENTITY
	var tint: Variant = Color.WHITE
	var stance: Variant = 0
	var alive: Variant = true
	var mixed: Variant = 0.0
	var health: Variant = 100
	var ticks: Array = []

	func _register_net_vars() -> void:
		replicate(&"position", DotNetVar.Type.VECTOR3_POSITION).interpolated()
		replicate(&"aim", DotNetVar.Type.FLOAT).interpolated()
		replicate(&"yaw", DotNetVar.Type.FLOAT).interpolated()
		replicate(&"rot", DotNetVar.Type.QUATERNION).interpolated()
		replicate(&"tint", DotNetVar.Type.FLOAT).interpolated()
		replicate(&"stance", DotNetVar.Type.UINT).bits(3).interpolated()
		replicate(&"alive", DotNetVar.Type.BOOL).interpolated()
		replicate(&"mixed", DotNetVar.Type.FLOAT).interpolated()
		replicate(&"health", DotNetVar.Type.UINT).bits(8)

	func _net_interpolated(tick: int) -> void:
		ticks.append(tick)

	## This frame's values and notifications; the notifications are then cleared.
	func state() -> PackedByteArray:
		var out := var_to_bytes([position, aim, yaw, rot, tint, stance, alive, mixed, health, ticks])
		ticks.clear()
		return out


## A behaviour that overrides the write: each value goes through the hook, which keeps its own copy.
class Hooked extends DotNetBehaviour:
	var velocity: Variant = Vector3.ZERO
	var spin: Variant = 0.0
	var writes: Array = []
	var write_count := 0

	func _register_net_vars() -> void:
		replicate(&"spin", DotNetVar.Type.FLOAT).interpolated()
		replicate(&"velocity", DotNetVar.Type.VECTOR3_VELOCITY).interpolated()

	func _net_write_property(property: StringName, value: Variant) -> void:
		writes.append([property, value])
		write_count += 1
		set(property, value)

	## This frame's values and hook calls; the calls are then cleared.
	func state() -> PackedByteArray:
		var out := var_to_bytes([velocity, spin, writes])
		writes.clear()
		return out


var _passed := 0
var _failed := 0
var _native := false


func _ready() -> void:
	DotLog.set_level(DotLog.Level.ERROR)
	_run.call_deferred()


func _run() -> void:
	print("dot-sim interp self-test")
	_native = DotSim.has(&"interp")
	print("  native interp module: %s" % ("loaded" if _native else "absent (script interpolator only)"))
	if OS.get_environment("DOT_SIM_REQUIRE_NATIVE") == "1":
		_check(_native, "the library is loaded (DOT_SIM_REQUIRE_NATIVE=1)")
	_test_fallback()
	if _native:
		for run_seed in [1, 2, 3, 4, 5, 6]:
			_test_stream(run_seed)
		_test_settings_off()
		_bench()
	print("")
	print("%d passed, %d failed" % [_passed, _failed])
	get_tree().quit(1 if _failed > 0 else 0)


func _check(ok: bool, what: String, detail: String = "") -> void:
	if ok:
		_passed += 1
		print("  ok    %s" % what)
	else:
		_failed += 1
		print("  FAIL  %s%s" % [what, "" if detail == "" else " (%s)" % detail])


func _config() -> DotNetConfig:
	var config := DotNetConfig.new()
	config.tick_rate = 64
	config.snapshot_rate = 32
	config.max_extrapolation_sec = 0.1
	config.resume_ease_max = 6.0
	config.resume_ease_sec = 0.12
	return config


func _interpolator(config: DotNetConfig, native: bool) -> DotNetInterpolator:
	var was := DotNetInterpolator.native_enabled
	DotNetInterpolator.native_enabled = native
	var interpolator := DotNetInterpolator.new(config)
	DotNetInterpolator.native_enabled = was
	return interpolator


## One side of a run: an interpolator and its entities.
class Rig:
	var interpolator: DotNetInterpolator
	var identities: Array[DotNetIdentity] = []

	func _init(p_interpolator: DotNetInterpolator, config: DotNetConfig, count: int) -> void:
		interpolator = p_interpolator
		for i in range(count):
			var identity := DotNetIdentity.new()
			identity.net_id = 10 + i
			var body := Body.new()
			var hooked := Hooked.new()
			body._bind(config)
			hooked._bind(config)
			var list: Array[DotNetBehaviour] = [body, hooked]
			identity.behaviours = list
			identities.append(identity)

	func state() -> PackedByteArray:
		var out := PackedByteArray()
		for identity in identities:
			for behaviour in identity.behaviours:
				out.append_array(behaviour.call(&"state"))
		return out

	func free_all() -> void:
		for identity in identities:
			for behaviour in identity.behaviours:
				behaviour.free()
			identity.free()


func _test_fallback() -> void:
	var config := _config()
	var script_only := _interpolator(config, false)
	_check(not script_only.is_native(), "native_enabled = false keeps the GDScript")
	config.sim_interp = false
	_check(not _interpolator(config, true).is_native(), "sim_interp = false keeps the GDScript")
	config.sim_interp = true
	_check(_interpolator(config, true).is_native() == _native, "the library is used exactly when loaded")


func _test_settings_off() -> void:
	var config := _config()
	config.sim_interp = false
	var interpolator := _interpolator(config, true)
	_check(interpolator.describe()["native"] == false, "describe() says when the GDScript draws")


## A random value of [param property] near [param base] (an entity's path), sometimes far.
func _values(rng: RandomNumberGenerator, tick: int, entity: int, partial: bool, teleport: bool) -> Dictionary:
	var t := float(tick)
	var r := 3.0 + float(entity)
	var centre := Vector3(100.0, 0.0, 0.0) * float(entity)
	var pos := centre + Vector3(cos(t * 0.05) * r, sin(t * 0.03), sin(t * 0.05) * r)
	if teleport:
		pos += Vector3(40.0, 0.0, -25.0)
	var all := {
		"position": pos,
		&"aim": Vector2(sin(t * 0.07), cos(t * 0.11)) * 2.5,
		&"yaw": fmod(t * 0.04, TAU) + rng.randf() * 0.01,
		&"rot": Quaternion(Vector3(rng.randf() - 0.5, 1.0, rng.randf() - 0.5).normalized(), rng.randf() * TAU),
		&"tint": Color(rng.randf(), rng.randf(), rng.randf(), 1.0),
		&"stance": rng.randi_range(0, 3),
		&"alive": rng.randf() < 0.9,
		# A float most of the time, an int now and then: a kind change between samples holds.
		&"mixed": rng.randi_range(0, 9) if rng.randf() < 0.2 else rng.randf() * 10.0,
		&"health": rng.randi_range(0, 100),
		&"spin": rng.randf_range(-3.0, 3.0),
		&"velocity": Vector3(rng.randf() - 0.5, 0.0, rng.randf() - 0.5) * 9.0,
		&"unknown": 7,
	}
	if not partial:
		return all
	var out := {}
	for key in all:
		if rng.randf() < 0.4:
			out[key] = all[key]
	return out


## Both sides fed the same pushes, frames and housekeeping; compared after every frame.
func _test_stream(run_seed: int) -> void:
	var config := _config()
	var rng := RandomNumberGenerator.new()
	rng.seed = run_seed
	var a := Rig.new(_interpolator(config, false), config, ENTITIES)
	var b := Rig.new(_interpolator(config, true), config, ENTITIES)
	var sides: Array[Rig] = [a, b]
	var loss := 0.05 * float(run_seed % 3)
	var queue: Array = []
	var next_send := 0
	var stalled_until := -1
	var teleported := {}
	var now := 0.0
	var first_bad := -1
	var bad_what := ""
	var drew := 0
	for frame in range(FRAMES):
		var dt: float = [1.0 / 144.0, 1.0 / 60.0, 1.0 / 30.0, 0.0][rng.randi_range(0, 3)] if run_seed % 2 == 0 else 1.0 / 144.0
		now += dt * float(config.tick_rate) if dt > 0.0 else 0.25
		# The server sends every other tick; some are lost, some late, some stalled for a while.
		while next_send <= int(now):
			if rng.randf() < 0.01:
				stalled_until = next_send + rng.randi_range(8, 30)
			if next_send >= stalled_until and rng.randf() >= loss:
				for e in range(ENTITIES):
					var teleport := rng.randf() < 0.004
					if teleport:
						teleported[e] = not teleported.get(e, false)
					var partial := rng.randf() < 0.3
					var vals := _values(rng, next_send, e, partial, teleported.get(e, false))
					var delay := rng.randi_range(0, 4) if rng.randf() < 0.3 else 0
					queue.append([int(now) + delay, next_send, 10 + e, vals])
					if rng.randf() < 0.05:
						# A retransmit of part of it, merged into the tick already held.
						queue.append([int(now) + delay + 1, next_send, 10 + e, _values(rng, next_send, e, true, teleported.get(e, false))])
			next_send += 2
		var due: Array = []
		var rest: Array = []
		for item in queue:
			(due if int(item[0]) <= int(now) else rest).append(item)
		queue = rest
		if run_seed % 3 == 1:
			due.shuffle()
		for item in due:
			for side in sides:
				side.interpolator.push(item[2], item[1], (item[3] as Dictionary).duplicate())
		for item in due:
			for side in sides:
				side.interpolator.note_lateness(item[1], int(now))
		for side in sides:
			side.interpolator.advance(dt)
		# Housekeeping now and then: an entity forgotten, the set pruned, a behaviour's plan grown.
		if frame == 700:
			for side in sides:
				side.interpolator.forget(11)
		if frame == 1100:
			for side in sides:
				side.interpolator.prune(PackedInt64Array([10, 11, 13]))
		if frame == 1500:
			for side in sides:
				var body := side.identities[0].behaviours[0] as Body
				body.replicate(&"health", DotNetVar.Type.UINT).interpolated()
		if frame == 1900:
			for side in sides:
				side.interpolator.clear()
		for side in sides:
			for identity in side.identities:
				side.interpolator.apply(identity, now)
		if first_bad < 0:
			var diff := _first_difference(a, b)
			if a.state() != b.state():
				first_bad = frame
				bad_what = diff
		drew += 1
	var label := "seed %d (loss %.2f%s): %d frames" % [run_seed, loss, ", reordered" if run_seed % 3 == 1 else "", drew]
	_check(first_bad < 0, label + " write for write", "first difference at frame %d: %s" % [first_bad, bad_what])
	var counts_a := [a.interpolator.sample_count(), a.interpolator.stall_count(), a.interpolator.extrapolation_count()]
	var counts_b := [b.interpolator.sample_count(), b.interpolator.stall_count(), b.interpolator.extrapolation_count()]
	_check(counts_a == counts_b and int(counts_a[2]) > 0, "seed %d counters %s" % [run_seed, counts_a], str(counts_b))
	var eased := 0
	for identity in a.identities:
		eased += (identity.behaviours[1] as Hooked).write_count
	_check(eased > 0, "seed %d write hook called %d times on both" % [run_seed, eased])
	for side in sides:
		side.interpolator.reset_counts()
	_check(b.interpolator.sample_count() == 0 and b.interpolator.track_count() == a.interpolator.track_count(), "seed %d reset and track count" % run_seed)
	a.free_all()
	b.free_all()


func _first_difference(a: Rig, b: Rig) -> String:
	for i in range(a.identities.size()):
		for j in range(a.identities[i].behaviours.size()):
			var x: Object = a.identities[i].behaviours[j]
			var y: Object = b.identities[i].behaviours[j]
			for property in [&"position", &"aim", &"yaw", &"rot", &"tint", &"stance", &"alive", &"mixed", &"health", &"ticks", &"velocity", &"spin", &"writes"]:
				if property in x and var_to_bytes(x.get(property)) != var_to_bytes(y.get(property)):
					return "entity %d behaviour %d %s: %s vs %s" % [i, j, property, str(x.get(property)).left(160), str(y.get(property)).left(160)]
	return "?"


## What one frame of [code]apply[/code] costs for 64 entities on each side (not a pass condition).
func _bench() -> void:
	var config := _config()
	var rng := RandomNumberGenerator.new()
	rng.seed = 99
	for native in [false, true]:
		var side := Rig.new(_interpolator(config, native), config, 64)
		for tick in range(0, 64, 2):
			for e in range(64):
				side.interpolator.push(10 + e, tick, _values(rng, tick, e, false, false))
		var now := 40.0
		var started := Time.get_ticks_usec()
		for frame in range(400):
			now += 64.0 / 144.0 * 0.05
			side.interpolator.advance(1.0 / 144.0)
			for identity in side.identities:
				side.interpolator.apply(identity, now)
		var ms := float(Time.get_ticks_usec() - started) / 1000.0 / 400.0
		print("  bench  apply, 64 entities, %s: %.3f ms per frame" % ["native" if native else "GDScript", ms])
		side.free_all()
