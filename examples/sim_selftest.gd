extends Node

## dot-sim's suite (`godot --headless --path . res://examples/sim_selftest.tscn`): each native module against the script it replaces, same inputs, same answers.
## With no library installed it checks that the script fallback runs and says the native half was skipped; CI's native-suites job installs it and sets , so a missing library fails.

const CHANNEL := "dot.sim.selftest"

## Random sight scans compared one by one.
const SCANS := 3000

## Bots and ticks of the whole-brain run, played once in script and once native.
const BOTS := 24
const TICKS := 600
const RATE := 30.0

var _passed := 0
var _failed := 0


func _ready() -> void:
	DotLog.set_level(DotLog.Level.ERROR)
	_run.call_deferred()


func _run() -> void:
	print("dot-sim self-test")
	var native := DotSim.has(&"bots")
	print("  native bots module: %s" % ("loaded" if native else "absent (script fallback only)"))
	if OS.get_environment("DOT_SIM_REQUIRE_NATIVE") == "1":
		_check(native, "the library is loaded (DOT_SIM_REQUIRE_NATIVE=1)")
	_test_fallback()
	if native:
		_test_scan_parity()
		_test_brain_parity()
		_bench_scan()
	if DotSim.has(&"arms"):
		_test_arms_parity()
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


func _test_fallback() -> void:
	print("fallback")
	var world := _world(_bodies(RandomNumberGenerator.new(), 12))
	world.contacts_version = -1
	_check(world.senses(world.contacts()) == null, "a world that does not bump contacts_version scans in script")
	DotBotWorld.native_enabled = false
	world.contacts_version = 1
	_check(world.senses(world.contacts()) == null, "native_enabled off scans in script")
	DotBotWorld.native_enabled = true
	var thread := Thread.new()
	thread.start(func() -> bool: return world.senses(world.contacts()) == null)
	_check(thread.wait_to_finish(), "a brain on a worker thread scans in script")
	var brain := DotBotBrain.new(1)
	var intent := brain.think(world.contacts()[0], world, 1.0)
	_check(intent != null, "a brain thinks with the script scan")


## Every scan of a random world: the same candidates in the same order, scores equal to the bit, the same target forgotten.
func _test_scan_parity() -> void:
	print("sight scan: native against script")
	var rng := RandomNumberGenerator.new()
	rng.seed = 7
	var mismatches := 0
	var first := ""
	var nonempty := 0
	for n in SCANS:
		var bodies := _bodies(rng, rng.randi_range(2, 64))
		var world := _world(bodies)
		world.contacts_version = n
		var me: DotBotContact = bodies[rng.randi_range(0, bodies.size() - 1)]
		var p := DotBotProfile.new()
		p.fov = rng.randf_range(40.0, 200.0)
		p.sight = rng.randf_range(5.0, 120.0)
		var forward := DotBotAim.direction(rng.randf_range(-180.0, 180.0), rng.randf_range(-85.0, 85.0))
		var target: int = (bodies[rng.randi_range(0, bodies.size() - 1)] as DotBotContact).id if rng.randf() < 0.6 else -1
		var visible := rng.randf() < 0.5
		var a := DotBotBrain.new(me.id)
		var b := DotBotBrain.new(me.id)
		for brain: DotBotBrain in [a, b]:
			brain.target_id = target
			brain._visible = visible
		var sa := PackedFloat32Array()
		var sb := PackedFloat32Array()
		var ca := a._candidates(me, bodies, p, me.eye(), forward, p.fov * 0.5, sa)
		var cb := b._candidates_native(world.senses(bodies), world, me, p, me.eye(), forward, p.fov * 0.5, sb)
		if not cb.is_empty():
			nonempty += 1
		var same := ca.size() == cb.size() and sa == sb and a.target_id == b.target_id and a._visible == b._visible
		if same:
			for i in ca.size():
				same = same and ca[i] == cb[i]
		if not same:
			mismatches += 1
			if first == "":
				first = "scan %d: script %s %s, native %s %s" % [n, _ids(ca), sa, _ids(cb), sb]
	_check(mismatches == 0, "%d random scans agree to the bit" % SCANS, first)
	_check(nonempty > SCANS / 4, "the scans are not trivially empty", str(nonempty))


## Whole brains over a moving fight, once in script and once native: every intent of every tick the same.
func _test_brain_parity() -> void:
	print("brains: native against script")
	DotBotWorld.native_enabled = false
	var script_run := _play()
	DotBotWorld.native_enabled = true
	var native_run := _play()
	var first := -1
	for t in mini(script_run.size(), native_run.size()):
		if script_run[t] != native_run[t]:
			first = t
			break
	_check(script_run.size() == TICKS and native_run.size() == TICKS, "both runs played every tick")
	_check(first < 0, "%d bots x %d ticks: every intent the same" % [BOTS, TICKS], "" if first < 0 else "tick %d: %s vs %s" % [first, script_run[first], native_run[first]])
	var fights := 0
	for line: String in script_run:
		fights += line.count("fight")
	_check(fights > BOTS * TICKS / 20, "the run has fights in it", str(fights))


## One fight of [constant BOTS] bots, two teams, pillars that block sight; one line of intents per tick.
func _play() -> PackedStringArray:
	var rng := RandomNumberGenerator.new()
	rng.seed = 11
	var bodies := []
	for i in BOTS:
		var c := DotBotContact.make(100 + i, Vector3(rng.randf_range(-30, 30), 0.0, rng.randf_range(-30, 30)), 1 + i % 2)
		c.view_yaw = rng.randf_range(-180.0, 180.0)
		c.view_pitch = 0.0
		bodies.append(c)
	var world := _world(bodies)
	var pillars: Array[Vector3] = []
	for i in 10:
		pillars.append(Vector3(rng.randf_range(-25, 25), 1.4, rng.randf_range(-25, 25)))
	world.line_of_sight_fn = func(a: Vector3, b: Vector3) -> bool:
		for q in pillars:
			if DotBotWorld.segment_hits_sphere(a, b, q, 2.0):
				return false
		return true
	world.waypoints_fn = func() -> PackedVector3Array: return PackedVector3Array([Vector3(-20, 0, -20), Vector3(20, 0, 20), Vector3(-20, 0, 20), Vector3(20, 0, -20)])
	var brains: Array[DotBotBrain] = []
	for c: DotBotContact in bodies:
		brains.append(DotBotBrain.new(c.id))
	var out := PackedStringArray()
	var dt := 1.0 / RATE
	for t in TICKS:
		var now := t * dt
		world.contacts_version = t
		var line := ""
		# Every bot thinks over the same bodies, then the tick applies them all, as a game does
		# (the bodies may not change between bumps of contacts_version).
		var intents: Array[DotBotIntent] = []
		for i in BOTS:
			var me: DotBotContact = bodies[i]
			var it := brains[i].think(me, world, now)
			intents.append(it)
			line += "%d:%s %.4f %.4f %s %s %d|" % [me.id, it.state, it.yaw, it.pitch, it.move, it.fire, it.target_id]
		for i in BOTS:
			var me: DotBotContact = bodies[i]
			var it := intents[i]
			me.view_yaw = it.yaw
			me.view_pitch = it.pitch
			var step := Vector3(it.move.x, 0.0, -it.move.y).rotated(Vector3.UP, deg_to_rad(it.yaw)) * 5.0 * dt
			me.velocity = step / dt
			me.position += step
			if it.fire and it.target_id >= 0:
				var hit := DotBotWorld.find(bodies, it.target_id)
				if hit != null and hit.alive:
					hit.health -= 0.02
					hit.alive = hit.health > 0.0
		for c: DotBotContact in bodies:
			if not c.alive and rng.randf() < 0.02:
				c.alive = true
				c.health = 1.0
		out.append(line)
	return out


## The cost of one scan, script and native, over the same worlds.
func _bench_scan() -> void:
	print("sight scan cost")
	var rng := RandomNumberGenerator.new()
	rng.seed = 3
	var bodies := _bodies(rng, 32)
	var world := _world(bodies)
	world.contacts_version = 1
	var p := DotBotProfile.new()
	var brain := DotBotBrain.new(1)
	var forward := DotBotAim.direction(30.0, 0.0)
	var reps := 20000
	var sc := PackedFloat32Array()
	var t0 := Time.get_ticks_usec()
	for i in reps:
		sc.clear()
		brain._candidates(bodies[i % 32], bodies, p, (bodies[i % 32] as DotBotContact).eye(), forward, p.fov * 0.5, sc)
	var gd := float(Time.get_ticks_usec() - t0) / reps
	var senses := world.senses(bodies)
	t0 = Time.get_ticks_usec()
	for i in reps:
		sc.clear()
		brain._candidates_native(senses, world, bodies[i % 32], p, (bodies[i % 32] as DotBotContact).eye(), forward, p.fov * 0.5, sc)
	var nat := float(Time.get_ticks_usec() - t0) / reps
	print("  32 bodies: script %.2f us/scan, native %.2f us/scan (%.1fx)" % [gd, nat, gd / maxf(nat, 0.001)])
	_check(nat < gd, "the native scan is cheaper than the script one")


func _bodies(rng: RandomNumberGenerator, n: int) -> Array:
	var out := []
	for i in n:
		var c := DotBotContact.make(i + 1, Vector3(rng.randf_range(-40, 40), rng.randf_range(-2, 6), rng.randf_range(-40, 40)), rng.randi_range(0, 2))
		c.alive = rng.randf() > 0.1
		c.aim_height = rng.randf_range(0.8, 1.4)
		c.eye_height = rng.randf_range(1.2, 1.8)
		out.append(c)
	return out


func _world(bodies: Array) -> DotBotWorld:
	var w := DotBotWorld.new()
	w.contacts_fn = func() -> Array: return bodies
	return w


func _ids(list: Array) -> String:
	var ids := PackedStringArray()
	for c: DotBotContact in list:
		ids.append(str(c.id))
	return ",".join(ids)


## dot-player-char's clearance scan, linked for the arms parity test (see README); absent, the test is skipped.
const ARMS_SCRIPT := "res://examples/linked/dot_view_arms_clear.gd"

## Random arm poses compared one by one.
const ARM_CASES := 4000


func _test_arms_parity() -> void:
	print("arms")
	if not ResourceLoader.exists(ARMS_SCRIPT):
		print("  skip  %s not linked" % ARMS_SCRIPT)
		return
	var clear: Script = load(ARMS_SCRIPT)
	var native: Object = ClassDB.instantiate(&"DotSimViewArms")
	var rng := RandomNumberGenerator.new()
	rng.seed = 20261113
	var same := 0
	var buried := 0
	var first_miss := ""
	for i: int in ARM_CASES:
		var c := _arm_case(rng)
		var want: PackedFloat64Array = clear.scan(c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], c[8])
		var got: PackedFloat64Array = native.clear_pole(c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], c[8])
		if got == want:
			same += 1
		elif first_miss == "":
			first_miss = "case %d: script %s native %s" % [i, want, got]
		if want[4] > 0.0 or want[3] != c[8] + (0.0 - c[8]) * 0.35:
			buried += 1
	_check(same == ARM_CASES, "%d random arm poses: the native scan matches the script to the bit" % ARM_CASES, first_miss)
	_check(buried > ARM_CASES / 4, "the poses reach the box (%d of %d turn or stay buried)" % [buried, ARM_CASES])
	var t0 := Time.get_ticks_usec()
	for i: int in 2000:
		var c := _arm_case(rng)
		clear.scan(c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], c[8])
	var t1 := Time.get_ticks_usec()
	for i: int in 2000:
		var c := _arm_case(rng)
		native.clear_pole(c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], c[8])
	var t2 := Time.get_ticks_usec()
	print("  bench 2000 scans (with case setup): script %d us, native %d us" % [t1 - t0, t2 - t1])


## One random arm: shoulder, hand target, bone lengths, pole, unit shoulder-to-hand axis, a rigid
## anchor transform, and a box in anchor space set around the arm so most poses touch it.
func _arm_case(rng: RandomNumberGenerator) -> Array:
	var shoulder := Vector3(rng.randf_range(-0.3, 0.3), rng.randf_range(1.2, 1.6), rng.randf_range(-0.2, 0.2))
	var lengths := Vector2(rng.randf_range(0.2, 0.35), rng.randf_range(0.2, 0.35))
	var reach := rng.randf_range(0.05, 1.05) * (lengths.x + lengths.y)
	var aim := Vector3(rng.randf_range(-1, 1), rng.randf_range(-1, 1), rng.randf_range(-1, 1))
	if aim.length_squared() < 0.01:
		aim = Vector3.FORWARD
	var target := shoulder + aim.normalized() * reach
	var dir := (target - shoulder).normalized()
	var pole := Vector3(rng.randf_range(-1, 1), rng.randf_range(-1, 1), rng.randf_range(-1, 1))
	var axis := Vector3(rng.randf_range(-1, 1), rng.randf_range(-1, 1), rng.randf_range(-1, 1))
	axis = axis.normalized() if axis.length_squared() > 0.01 else Vector3.UP
	var to_anchor := Transform3D(Basis(axis, rng.randf_range(-PI, PI)), Vector3(rng.randf_range(-2, 2), rng.randf_range(-2, 2), rng.randf_range(-2, 2)))
	var mid := to_anchor * shoulder.lerp(target, rng.randf_range(0.0, 1.0))
	var size := Vector3(rng.randf_range(0.02, 0.4), rng.randf_range(0.02, 0.4), rng.randf_range(0.02, 0.4))
	var box := AABB(mid - size * rng.randf_range(0.0, 1.0), size)
	var up := Vector3(rng.randf_range(-0.2, 0.2), 1.0, rng.randf_range(-0.2, 0.2)).normalized()
	return [shoulder, target, lengths, pole, dir, to_anchor, box, up, rng.randf_range(-80.0, 80.0)]
