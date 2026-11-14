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
		_test_bodies_parity()
		_bench_bodies()
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


## The column table (load_bodies) against the contact objects (_pack) and the game's script near test.
func _test_bodies_parity() -> void:
	print("column table")
	var rng := RandomNumberGenerator.new()
	rng.seed = 11
	var p := DotBotProfile.new()
	var mismatches := 0
	var near_bad := 0
	var near_hits := 0
	var first := ""
	for w in 200:
		var bodies := _bodies(rng, rng.randi_range(1, 64))
		var world := _world(bodies)
		world.contacts_version = w
		var packed: Object = world.senses(bodies)
		var cols: Object = ClassDB.instantiate(&"DotSimBotSenses")
		_load_columns(cols, bodies)
		var me: DotBotContact = bodies[rng.randi() % bodies.size()]
		for q in 10:
			var sight := rng.randf_range(5.0, 120.0)
			var target: int = (bodies[rng.randi() % bodies.size()] as DotBotContact).id if rng.randf() < 0.6 else -1
			var visible := rng.randf() < 0.5
			var forward := DotBotAim.direction(rng.randf_range(-180, 180), rng.randf_range(-30, 30))
			var a: PackedInt32Array = packed.call(&"candidates", me.id, me.team, me.eye(), forward, p.fov * 0.5, sight, 2.0, target, visible)
			var sa: PackedFloat32Array = packed.call(&"scores")
			var b: PackedInt32Array = cols.call(&"candidates", me.id, me.team, me.eye(), forward, p.fov * 0.5, sight, 2.0, target, visible)
			var sb: PackedFloat32Array = cols.call(&"scores")
			if a != b or sa != sb or packed.call(&"target_dead") != cols.call(&"target_dead"):
				mismatches += 1
				if first == "":
					first = "world %d: %s vs %s" % [w, a, b]
		var near := rng.randf_range(5.0, 40.0)
		var flags: PackedByteArray = cols.call(&"nobody_near", near)
		for i in bodies.size():
			var want := _nobody_near_script(bodies[i], bodies, near)
			near_hits += 0 if want else 1
			if flags[i] != (1 if want else 0):
				near_bad += 1
	_check(mismatches == 0, "2000 scans over load_bodies equal the ones over the packed contacts", first)
	_check(near_bad == 0, "the native near test agrees with the script one on every body", str(near_bad))
	_check(near_hits > 100, "the near test is not trivially true", str(near_hits))
	var sight: Object = ClassDB.instantiate(&"DotSimBotSenses")
	_load_columns(sight, _bodies(rng, 8))
	sight.call(&"load_contacts", PackedInt64Array([1]), PackedVector3Array([Vector3.ZERO]), PackedInt32Array([0]), PackedByteArray([1]))
	_check((sight.call(&"nobody_near", 10.0) as PackedByteArray).is_empty(), "load_contacts drops the columns of an earlier load_bodies")


## The game's _nobody_near over all contacts (game-fatal-contact arena_bots.gd), the script answer.
static func _nobody_near_script(me: DotBotContact, all: Array, near: float) -> bool:
	var near_sq := near * near
	for item: DotBotContact in all:
		if item.alive and item.id != me.id and me.is_enemy(item) and item.position.distance_squared_to(me.position) < near_sq:
			return false
	return true


## The game's near test as it runs at 32 players: live contacts filed by cell, 3x3 cells per bot.
static func _nobody_near_cells(me: DotBotContact, cells: Dictionary, near: float) -> bool:
	var near_sq := near * near
	var cx := int(floorf(me.position.x / near)) + 4096
	var cz := int(floorf(me.position.z / near)) + 4096
	for dx in range(-1, 2):
		for dz in range(-1, 2):
			var cell: Variant = cells.get((cx + dx) * 8192 + cz + dz)
			if cell == null:
				continue
			for item: DotBotContact in cell:
				if item.id != me.id and me.is_enemy(item) and item.position.distance_squared_to(me.position) < near_sq:
					return false
	return true


## Columns read back from contact objects: the worst case, a game that keeps no columns of its own.
static func _load_columns(senses: Object, bodies: Array) -> void:
	var ids := PackedInt64Array()
	var pos := PackedVector3Array()
	var teams := PackedInt32Array()
	var alive := PackedByteArray()
	var eyes := PackedFloat32Array()
	var aims := PackedFloat32Array()
	for c: DotBotContact in bodies:
		ids.append(c.id)
		pos.append(c.position)
		teams.append(c.team)
		alive.append(1 if c.alive else 0)
		eyes.append(c.eye_height)
		aims.append(c.aim_height)
	senses.call(&"load_bodies", ids, pos, teams, alive, eyes, aims)


## One tick of the contact work FC does for 32 players and 31 bots, script against columns:
## script = _pack + the near cells + 31 near tests; columns = the columns appended + load_bodies + one near call.
func _bench_bodies() -> void:
	print("column table cost (one tick, 32 bodies, 31 near tests)")
	var rng := RandomNumberGenerator.new()
	rng.seed = 5
	var bodies := _bodies(rng, 32)
	for c: DotBotContact in bodies:
		c.position.y = 0.0
	var world := _world(bodies)
	var near := 30.0
	var reps := 2000
	var sink := 0
	var t0 := Time.get_ticks_usec()
	for r in reps:
		world.contacts_version = r
		world.senses(bodies)
		var cells := {}
		for c: DotBotContact in bodies:
			if c.alive:
				var k := (int(floorf(c.position.x / near)) + 4096) * 8192 + int(floorf(c.position.z / near)) + 4096
				var cell: Variant = cells.get(k)
				if cell == null:
					cells[k] = [c]
				else:
					(cell as Array).append(c)
		for i in 31:
			sink += 1 if _nobody_near_cells(bodies[i], cells, near) else 0
	var gd := float(Time.get_ticks_usec() - t0) / reps
	var cols: Object = ClassDB.instantiate(&"DotSimBotSenses")
	t0 = Time.get_ticks_usec()
	for r in reps:
		_load_columns(cols, bodies)
		var flags: PackedByteArray = cols.call(&"nobody_near", near)
		for i in 31:
			sink += flags[i]
	var worst := float(Time.get_ticks_usec() - t0) / reps
	# The game keeps the columns as it writes the contacts: only the appends are new work.
	var src_pos := PackedVector3Array()
	for c: DotBotContact in bodies:
		src_pos.append(c.position)
	var ids := PackedInt64Array()
	var pos := PackedVector3Array()
	var teams := PackedInt32Array()
	var alive := PackedByteArray()
	var eyes := PackedFloat32Array()
	var aims := PackedFloat32Array()
	t0 = Time.get_ticks_usec()
	for r in reps:
		ids.clear(); pos.clear(); teams.clear(); alive.clear(); eyes.clear(); aims.clear()
		for i in 32:
			ids.append(i + 1)
			pos.append(src_pos[i])
			teams.append(i % 3)
			alive.append(1)
			eyes.append(1.6)
			aims.append(1.25)
		cols.call(&"load_bodies", ids, pos, teams, alive, eyes, aims)
		var flags: PackedByteArray = cols.call(&"nobody_near", near)
		for i in 31:
			sink += flags[i]
	var inline := float(Time.get_ticks_usec() - t0) / reps
	print("  script %.1f us, columns read from contacts %.1f us (%.1fx), columns kept by the game %.1f us (%.1fx)  [%d]" % [gd, worst, gd / maxf(worst, 0.001), inline, gd / maxf(inline, 0.001), sink % 7])
	_check(worst < gd, "the column table is cheaper than the script pack and near test")


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
