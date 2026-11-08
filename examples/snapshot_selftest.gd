extends Node

## The snapshot module's suite ([code]godot --headless --path . res://examples/snapshot_selftest.tscn[/code]):
## dot-net's state codec in GDScript and in [code]DotSimSnapshotCodec[/code], the same inputs, the same
## bytes, values, errors and reader positions. Then whole servers and clients, codec off and on, over
## lossy links at 22 and 33 Hz with interest rings: every payload and every value received equal.
## With no library it checks the fallback and says the native half was skipped; with
## [code]DOT_SIM_REQUIRE_NATIVE=1[/code] (CI) a missing library fails instead.

const ROUNDS := 4000
const NOISE := 1500


## Every wire type, and the cases the codec hands back to the script.
class Every extends DotNetBehaviour:
	var b: Variant = false
	var u3: Variant = 0
	var u64: Variant = 0
	var i7: Variant = 0
	var i32: Variant = 0
	var vi: Variant = 0
	var f: Variant = 0.0
	var fr: Variant = 0.0
	var fr40: Variant = 0.0
	var pos: Variant = Vector3.ZERO
	var vel: Variant = Vector3.ZERO
	var vr: Variant = Vector3.ZERO
	var dir: Variant = Vector3.FORWARD
	var q: Variant = Quaternion.IDENTITY
	var ang: Variant = 0.0
	var ang16: Variant = 0.0
	var s: Variant = ""
	var by: Variant = PackedByteArray()
	var c: Variant = 0
	var rated: Variant = 0
	var log: Array = []

	func _register_net_vars() -> void:
		replicate(&"b", DotNetVar.Type.BOOL)
		replicate(&"u3", DotNetVar.Type.UINT).bits(3)
		replicate(&"u64", DotNetVar.Type.UINT).bits(64)
		replicate(&"i7", DotNetVar.Type.INT).bits(7)
		replicate(&"i32", DotNetVar.Type.INT).bits(32)
		replicate(&"vi", DotNetVar.Type.VARINT)
		replicate(&"f", DotNetVar.Type.FLOAT)
		replicate(&"fr", DotNetVar.Type.FLOAT_RANGE).range_of(-5.0, 20.0).bits(13)
		replicate(&"fr40", DotNetVar.Type.FLOAT_RANGE).range_of(0.0, 1.0).bits(40)
		replicate(&"pos", DotNetVar.Type.VECTOR3_POSITION)
		replicate(&"vel", DotNetVar.Type.VECTOR3_VELOCITY)
		replicate(&"vr", DotNetVar.Type.VECTOR3_RANGE).range_of(-3.0, 3.0).bits(10)
		replicate(&"dir", DotNetVar.Type.DIRECTION).bits(12)
		replicate(&"q", DotNetVar.Type.QUATERNION)
		replicate(&"ang", DotNetVar.Type.ANGLE).bits(9)
		replicate(&"ang16", DotNetVar.Type.ANGLE).bits(16)
		replicate(&"s", DotNetVar.Type.STRING)
		replicate(&"by", DotNetVar.Type.BYTES)
		replicate_custom(&"c", _write_c, _read_c)
		replicate(&"rated", DotNetVar.Type.UINT).bits(12).at_most(10.0)
		for declaration in net_vars:
			var property := declaration.property
			var _h := declaration.on_change(func(previous: Variant, value: Variant) -> void:
				log.append([property, previous, value]))

	func _write_c(writer: DotNetWriter, value: Variant) -> void:
		writer.write_uint(int(value) & 31, 5)
		writer.write_float32(float(value) * 0.5)

	func _read_c(reader: DotNetReader) -> Variant:
		var low := reader.read_uint(5)
		var _half := reader.read_float32()
		return low


## A player as arena sends it: what a client applies 128 of per snapshot.
class Player extends DotNetBehaviour:
	var position := Vector3.ZERO
	var velocity := Vector3.ZERO
	var yaw := 0.0
	var pitch := 0.0
	var health := 100.0
	var stance := 0
	var weapon := 0
	var ammo := 30
	var points := 0
	var flags := 0
	var grounded := true
	var tag := ""
	var spin := Quaternion.IDENTITY
	var changes := 0

	func _register_net_vars() -> void:
		replicate(&"position", DotNetVar.Type.VECTOR3_POSITION).interpolated()
		replicate(&"velocity", DotNetVar.Type.VECTOR3_VELOCITY)
		replicate(&"yaw", DotNetVar.Type.ANGLE).bits(12)
		replicate(&"pitch", DotNetVar.Type.ANGLE).bits(10)
		replicate(&"health", DotNetVar.Type.FLOAT_RANGE).range_of(0.0, 2000.0).bits(14)
		replicate(&"stance", DotNetVar.Type.UINT).bits(3)
		replicate(&"weapon", DotNetVar.Type.UINT).bits(5)
		replicate(&"ammo", DotNetVar.Type.UINT).bits(9).to_owner_only()
		replicate(&"points", DotNetVar.Type.UINT).bits(16)
		replicate(&"flags", DotNetVar.Type.UINT).bits(8)
		replicate(&"grounded", DotNetVar.Type.BOOL)
		replicate(&"tag", DotNetVar.Type.STRING)
		replicate(&"spin", DotNetVar.Type.QUATERNION)
		for declaration in net_vars:
			var _h := declaration.on_change(func(_p: Variant, _v: Variant) -> void: changes += 1)


## One direction of a seeded link: loses, repeats and delays (so reorders).
class Link:
	var rng := RandomNumberGenerator.new()
	var loss := 0.0
	var duplicate := 0.0
	var jitter := 0
	var queue: Array = []

	func push(tick: int, payload: PackedByteArray) -> void:
		if rng.randf() < loss:
			return
		queue.append([tick + rng.randi_range(0, jitter), payload])
		if rng.randf() < duplicate:
			queue.append([tick + rng.randi_range(0, jitter), payload])

	func due(tick: int) -> Array:
		var out: Array = []
		var rest: Array = []
		for item in queue:
			(out if int(item[0]) <= tick else rest).append(item)
		queue = rest
		out.sort_custom(func(a: Array, b: Array) -> bool: return int(a[0]) < int(b[0]))
		return out


var _passed := 0
var _failed := 0
var _native := false
var _config := DotNetConfig.new()


func _ready() -> void:
	DotLog.set_level(DotLog.Level.ERROR)
	_run.call_deferred()


func _run() -> void:
	print("dot-sim snapshot self-test")
	_native = DotSim.has(&"snapshot")
	print("  native snapshot module: %s" % ("loaded" if _native else "absent (script codec only)"))
	if OS.get_environment("DOT_SIM_REQUIRE_NATIVE") == "1":
		_check(_native, "the library is loaded (DOT_SIM_REQUIRE_NATIVE=1)")
	# Not the defaults: the spec must carry the config's grid, speed and rotation widths.
	_config.world_extent = 1000.0
	_config.max_speed = 37.0
	_config.velocity_bits = 11
	_config.rotation_bits = 10
	_test_fallback()
	if _native:
		_test_encode_parity()
		_test_decode_parity()
		_test_wire(22, 22, 0.0, 0, false, 0)
		_test_wire(33, 33, 0.2, 3, false, 0)
		_test_wire(22, 22, 0.15, 2, true, 0)
		_test_wire(33, 33, 0.2, 3, true, 2)
		_test_wire(66, 33, 0.1, 2, true, 0)
		_bench()
		_golden()
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


func _every(codec: bool) -> Every:
	var e := Every.new()
	e._bind(_config)
	if not codec:
		e._sim_codec = null
	return e


func _test_fallback() -> void:
	print("fallback")
	DotNetSimCodec.enabled = false
	_check(_every(true)._sim_codec == null, "DotNetSimCodec.enabled off: the script codec")
	DotNetSimCodec.enabled = true
	var off := DotNetConfig.new()
	off.sim_codec = false
	var e := Every.new()
	e._bind(off)
	_check(e._sim_codec == null, "DotNetConfig.sim_codec off: the script codec")
	_check(DotNetSimCodec.available() == _native, "available() follows the library", str(_native))
	_check((_every(true)._sim_codec != null) == _native, "a behaviour binds the codec exactly when it can load")
	var script := _every(false)
	script.u3 = 5
	script.s = "fallback"
	var decls: Array[DotNetVar] = [script.net_vars[1], script.net_vars[16]]
	var encoded := script._encode(decls, 3)
	var back := _every(false)
	var reader := DotNetReader.new(encoded[0])
	reader.seek_bits(3)
	var read := back.read_state(reader, 1)
	_check(read.ok and back.u3 == 5 and back.s == "fallback", "the script codec round-trips")


func _value(rng: RandomNumberGenerator, d: DotNetVar) -> Variant:
	var edge := rng.randi_range(0, 9) == 0
	match d.type:
		DotNetVar.Type.BOOL:
			return [true, false, 0, 1, 2, 0.0, 0.5, -3][rng.randi_range(0, 7)]
		DotNetVar.Type.UINT, DotNetVar.Type.INT, DotNetVar.Type.VARINT:
			if edge:
				return [0, -1, 1 << 62, -(1 << 63), (1 << 63) - 1, 2.9, -2.9, 1.0e300, NAN, true, "12"][rng.randi_range(0, 10)]
			return rng.randi_range(-100000, 100000) >> rng.randi_range(0, 16)
		DotNetVar.Type.FLOAT, DotNetVar.Type.FLOAT_RANGE, DotNetVar.Type.ANGLE:
			if edge:
				return [0.0, -0.0, NAN, INF, -INF, 1.0e39, -1.0e-45, 7, true, 360.0, -720.0, 19.999999][rng.randi_range(0, 11)]
			return rng.randf_range(-400.0, 400.0)
		DotNetVar.Type.VECTOR3_POSITION, DotNetVar.Type.VECTOR3_VELOCITY, DotNetVar.Type.VECTOR3_RANGE, DotNetVar.Type.DIRECTION:
			if edge:
				return [Vector3.ZERO, Vector3(INF, -INF, NAN), Vector3(1e9, -1e9, 0.0), Vector3(0, -1, 0), Vector3(1e-30, 0, 0)][rng.randi_range(0, 4)]
			var scale := [1.0, 50.0, 2000.0][rng.randi_range(0, 2)] as float
			return Vector3(rng.randf_range(-scale, scale), rng.randf_range(-scale, scale), rng.randf_range(-scale, scale))
		DotNetVar.Type.QUATERNION:
			if edge:
				return [Quaternion.IDENTITY, Quaternion(0, 0, 0, 0), Quaternion(0, 0, 0, -1), Quaternion(1, 2, 3, 4), Quaternion(0.5, 0.5, 0.5, 0.5), Quaternion(0, 0.7071067811865476, 0, 0.7071067811865476)][rng.randi_range(0, 5)]
			var axis := Vector3(rng.randf_range(-1, 1), rng.randf_range(-1, 1), rng.randf_range(-1, 1))
			if axis.is_zero_approx():
				axis = Vector3.UP
			return Quaternion(axis.normalized(), rng.randf_range(-TAU, TAU))
		DotNetVar.Type.STRING:
			return ["", "a", "héllo", "x".repeat(rng.randi_range(0, 40))][rng.randi_range(0, 3)]
		DotNetVar.Type.BYTES:
			var out := PackedByteArray()
			for i in rng.randi_range(0, 9 if not edge else 300):
				out.append(rng.randi_range(0, 255))
			return out
	return rng.randi_range(0, 1000)


## The same declarations, values and pad: script and codec write the same bytes, end bit,
## recorded values and rate-limited properties.
func _test_encode_parity() -> void:
	print("encode: codec against script, every type, pads 0..7")
	var rng := RandomNumberGenerator.new()
	rng.seed = 2026
	var script := _every(false)
	var native := _every(true)
	var stranger := DotNetVar.make(&"nobody", DotNetVar.Type.UINT)
	var mismatches := 0
	var first := ""
	var native_hits := 0
	var per_type := {}
	for n in ROUNDS:
		var decls: Array[DotNetVar] = []
		for d in native.net_vars:
			if rng.randf() < 0.4:
				decls.append(d)
		if rng.randi_range(0, 7) == 0:
			decls.shuffle()
		if rng.randi_range(0, 30) == 0:
			decls.insert(rng.randi_range(0, decls.size()), stranger)
		for d in decls:
			if d == stranger:
				continue
			var v: Variant = _value(rng, d)
			script.set(d.property, v)
			native.set(d.property, v)
		var pad := n % 8
		var script_decls: Array[DotNetVar] = []
		for d in decls:
			script_decls.append(d if d == stranger else script.net_vars[native.net_vars.find(d)])
		var a := script._encode(script_decls, pad)
		var direct := DotNetSimCodec.encode(native, decls, pad)
		var b := native._encode(decls, pad)
		if not direct.is_empty():
			native_hits += 1
			for d in decls:
				per_type[d.type] = true
		# The recorded values are the same objects either way; the bytes are the claim.
		if a[0] != b[0] or a[1] != b[1] or var_to_bytes(a[2]) != var_to_bytes(b[2]) or a[3] != b[3]:
			mismatches += 1
			if first == "":
				first = "round %d pad %d: %s vs %s" % [n, pad, (a[0] as PackedByteArray).hex_encode(), (b[0] as PackedByteArray).hex_encode()]
	_check(mismatches == 0, "%d encodings equal to the bit" % ROUNDS, first)
	# A state with any script-only declaration (fr40, s, c: each in 40% of draws) is the
	# script's whole, so about 0.6^3 of the rounds are the codec's.
	_check(native_hits > ROUNDS / 6, "the codec wrote %d of them itself" % native_hits)
	var types := 0
	for t in [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13]:
		types += 1 if per_type.has(t) else 0
	_check(types == 13, "every built-in type but string went through the codec (%d of 13)" % types)
	# A value of a type only the script converts hands the whole state back.
	native.i32 = "12"
	var only: Array[DotNetVar] = [native.net_vars[4]]
	_check(DotNetSimCodec.encode(native, only, 0).is_empty(), "a string for an INT is the script's")
	var thread := Thread.new()
	native.u3 = 3
	var one: Array[DotNetVar] = [native.net_vars[1]]
	thread.start(func() -> bool: return DotNetSimCodec.encode(native, one, 0).is_empty())
	_check(thread.wait_to_finish(), "a snapshot worker thread encodes in script")


## A receiver's whole visible result of one read.
func _read_result(e: Every, bytes: PackedByteArray, pad: int, limit: int) -> PackedByteArray:
	e.log.clear()
	var reader := DotNetReader.new(bytes)
	if limit >= 0:
		reader.limit_bits(limit)
	reader.seek_bits(pad)
	var r := e.read_state(reader, 7)
	var values := {}
	for d in e.net_vars:
		values[d.property] = e.get(d.property)
	var err: Array = [] if r.ok else [r.error.code, r.error.message, r.error.detail]
	return var_to_bytes([r.ok, r.value if r.ok else null, err, reader._bit_pos, reader.exhausted, values, e.received_values(), e.log])


## The same packets into a script and a codec receiver: the same values, change handlers,
## errors and reader position, cut anywhere and filled with noise.
func _test_decode_parity() -> void:
	print("decode: codec against script on the same packets")
	var rng := RandomNumberGenerator.new()
	rng.seed = 33
	var writer := _every(false)
	var script := _every(false)
	var native := _every(true)
	var packets: Array = []
	for n in ROUNDS:
		var decls: Array[DotNetVar] = []
		for d in writer.net_vars:
			if rng.randf() < 0.4:
				decls.append(d)
				writer.set(d.property, _value(rng, d))
		if rng.randi_range(0, 9) == 0:
			decls.shuffle()
		var pad := n % 8
		var e := writer._encode(decls, pad)
		packets.append([e[0], pad, int(e[1])])
	var mismatches := 0
	var first := ""
	var cuts := 0
	var errors := 0
	for n in packets.size():
		var p: Array = packets[n]
		var limits := [-1]
		if n % 4 == 0:
			limits.append(rng.randi_range(int(p[1]), int(p[2])))
		for limit: int in limits:
			var a := _read_result(script, p[0], p[1], limit)
			var b := _read_result(native, p[0], p[1], limit)
			cuts += 1 if limit >= 0 else 0
			errors += 0 if bytes_to_var(a)[0] else 1
			if a != b:
				mismatches += 1
				if first == "":
					first = "packet %d limit %d: %s vs %s" % [n, limit, bytes_to_var(a), bytes_to_var(b)]
	_check(mismatches == 0, "%d reads equal, %d of them cut short (%d errors)" % [packets.size() + cuts, cuts, errors], first)

	mismatches = 0
	first = ""
	errors = 0
	for n in NOISE:
		var noise := PackedByteArray()
		for i in rng.randi_range(0, 48):
			noise.append(rng.randi_range(0, 255))
		var pad := rng.randi_range(0, 7)
		var a := _read_result(script, noise, pad, -1)
		var b := _read_result(native, noise, pad, -1)
		errors += 0 if bytes_to_var(a)[0] else 1
		if a != b:
			mismatches += 1
			if first == "":
				first = "noise %s: %s vs %s" % [noise.hex_encode(), bytes_to_var(a), bytes_to_var(b)]
	_check(mismatches == 0, "%d noise packets read alike (%d errors)" % [NOISE, errors], first)
	var exhausted := DotNetReader.new(PackedByteArray([0xff]))
	var _x := exhausted.read_bits(16)
	var r1 := script.read_state(exhausted, 1)
	exhausted.exhausted = true
	var r2 := native.read_state(exhausted, 1)
	_check(not r1.ok and not r2.ok and r1.error.message == r2.error.message, "an exhausted reader fails alike")


## Whole servers and clients over a seeded link, run with the codec off and on. Every payload
## the server sent and everything each client received must be equal.
func _test_wire(tick_rate: int, snapshot_rate: int, loss: float, jitter: int, rings: bool, workers: int) -> void:
	var label := "%d Hz ticks, %d Hz snapshots, loss %d%%%s%s" % [tick_rate, snapshot_rate, int(loss * 100.0),
		", interest rings" if rings else "", ", %d workers" % workers if workers > 0 else ""]
	print("wire: %s" % label)
	var off := _wire_run(false, tick_rate, snapshot_rate, loss, jitter, rings, workers)
	var on := _wire_run(true, tick_rate, snapshot_rate, loss, jitter, rings, workers)
	_check(int(on["codecs"]) > 0 and int(off["codecs"]) == 0, "the codec bound on server and clients (%d behaviours)" % on["codecs"])
	_check(on["sent"] == off["sent"] and int(on["payloads"]) > 0,
		"%d payloads, every byte equal (%s)" % [on["payloads"], on["sent"]])
	_check(on["seen"] == off["seen"] and int(on["applied"]) > 0,
		"%d snapshots applied, every value and change handler equal (%s)" % [on["applied"], on["seen"]],
		"first differing tick %d" % _first_diff(off["ticks"], on["ticks"]))
	_check(int(on["failed"]) == 0 and int(off["failed"]) == 0, "no snapshot refused (%d, %d)" % [on["failed"], off["failed"]])


func _first_diff(a: Array, b: Array) -> int:
	for i in mini(a.size(), b.size()):
		if a[i] != b[i]:
			return i + 1
	return -1 if a.size() == b.size() else mini(a.size(), b.size()) + 1


func _wire_manager(label: String, server: bool, peer: int, tick_rate: int, snapshot_rate: int) -> DotNetManager:
	var m := DotNetManager.new()
	m.name = label
	m.is_server = server
	m.service_scope = StringName(label.to_lower())
	m.local_peer_id = peer
	m.auto_tick = false
	m.config_file = ""
	m.config = DotNetConfig.new()
	m.config.tick_rate = tick_rate
	m.config.snapshot_rate = snapshot_rate
	m.config.per_client_budget = 0
	# dot-net's own native core encodes in C++; off, so the script encode (and the codec) runs.
	m.config.native = false
	add_child(m)
	return m


func _player_body(_context: Dictionary) -> Node:
	var body := Node3D.new()
	body.name = "Body"
	var identity := DotNetIdentity.new()
	identity.name = "NetIdentity"
	identity.prefab_id = &"player"
	identity.authority = DotNetIdentity.Authority.SERVER
	identity.interest_linger_sec = 0.0
	body.add_child(identity)
	var player := Player.new()
	player.name = "Player"
	body.add_child(player)
	identity.entity = body
	identity.behaviours = [player]
	player.identity = identity
	player._bind(DotNetConfig.new())
	return body


func _wire_run(codec: bool, tick_rate: int, snapshot_rate: int, loss: float, jitter: int, rings: bool, workers: int) -> Dictionary:
	const PEERS := 3
	const ENTITIES := 40
	const TICKS := 160
	DotNetSimCodec.enabled = codec
	var tag := "%s%d_%d_%d" % ["On" if codec else "Off", tick_rate, snapshot_rate, workers]
	var host := _wire_manager("WireServer" + tag, true, 1, tick_rate, snapshot_rate)
	host.config.snapshot_workers = workers
	if rings:
		var distance := DotNetInterestDistance.new()
		distance.radius = 45.0
		distance.far_distance = 15.0
		distance.far_every = 2
		distance.far_tiers = 2
		distance.evaluation_interval_sec = 0.0
		host.interest = distance
	var clients: Array[DotNetManager] = []
	var links: Array[Link] = []
	var acks: Array[Link] = []
	for i in PEERS:
		clients.append(_wire_manager("WireClient%s_%d" % [tag, i], false, 2 + i, tick_rate, snapshot_rate))
		for list: Array[Link] in [links, acks]:
			var link := Link.new()
			link.rng.seed = 91 * (i + 1) + (5 if list == acks else 0)
			link.loss = loss
			link.duplicate = loss * 0.5
			link.jitter = jitter
			list.append(link)
	var _s := host.setup()
	for client in clients:
		var _c := client.setup()
	var tick := [0]
	var sent := HashingContext.new()
	var _h := sent.start(HashingContext.HASH_MD5)
	var payloads := [0]
	host.send_fn = func(peer_id: int, payload: PackedByteArray, d: int) -> void:
		var index := peer_id - 2
		if index < 0 or index >= PEERS:
			return
		if d != DotNetMessage.Delivery.UNRELIABLE:
			var _r := clients[index].receive(payload, 1)
			return
		payloads[0] += 1
		var _u := sent.update(PackedByteArray([peer_id]))
		var _v := sent.update(payload)
		links[index].push(tick[0], payload)
	host.start()
	for client in clients:
		client.start()
		client.spawner.register_factory(&"player", _player_body)
	host.spawner.register_factory(&"player", _player_body)
	for i in PEERS:
		host.add_peer(2 + i)
	var server_side: Array[Player] = []
	for i in PEERS + ENTITIES:
		var owner := 2 + i if i < PEERS else 0
		var made: DotNetIdentity = host.spawner.spawn(&"player", owner, Transform3D.IDENTITY, 0).value
		server_side.append(made.behaviours[0] as Player)
		for client in clients:
			var _m := client.spawner.spawn_remote(&"player", made.net_id, owner, Transform3D.IDENTITY, 0)
	var codecs := 0
	var seen := HashingContext.new()
	var _g := seen.start(HashingContext.HASH_MD5)
	var ticks: Array = []
	var applied := 0
	var failed := 0
	for t in range(1, TICKS + 1):
		tick[0] = t
		for i in server_side.size():
			var p := server_side[i]
			var a := float(i) * 0.61 + float(t) * (0.03 if i >= PEERS else 0.07)
			var r := 5.0 + float(i % 11) * 6.0
			p.position = Vector3(cos(a) * r, float(i % 3), sin(a) * r)
			(p.identity.get_parent() as Node3D).position = p.position
			p.velocity = Vector3(-sin(a), 0.0, cos(a)) * 3.0
			p.yaw = fposmod(a * 57.0, 360.0)
			p.pitch = float((t + i) % 40) - 20.0
			p.spin = Quaternion(Vector3.UP, a)
			if (t + i) % 9 == 0:
				p.health = 100.0 - float((t * 7 + i) % 90)
				p.ammo = (t + i) % 30
			if (t + i) % 13 == 0:
				p.points += 10
				p.flags = (t * i) & 255
				p.grounded = not p.grounded
			if (t + i) % 31 == 0:
				p.tag = "t%d" % t
		host.server_tick(t)
		var row := PackedByteArray()
		for index in PEERS:
			for item in links[index].due(t):
				var result := clients[index].receive_snapshot(item[1])
				applied += 1
				failed += 0 if result.ok else 1
				acks[index].push(t, clients[index].encode_ack())
			for item in acks[index].due(t):
				var _a := host.receive_ack_payload(2 + index, item[1])
			var state := []
			var ids := clients[index].registry.all()
			ids.sort_custom(func(x: DotNetIdentity, y: DotNetIdentity) -> bool: return x.net_id < y.net_id)
			for identity in ids:
				var player := identity.behaviours[0] as Player
				state.append([identity.net_id, player.received_values(), player.changes])
			row.append_array(var_to_bytes(state))
		var _u := seen.update(row)
		ticks.append(row)
	for p in server_side:
		codecs += 1 if p._sim_codec != null else 0
	for client in clients:
		for identity in client.registry.all():
			codecs += 1 if (identity.behaviours[0] as DotNetBehaviour)._sim_codec != null else 0
	host.stop()
	for client in clients:
		client.stop()
	for node: Node in [host] + clients:
		remove_child(node)
		node.free()
	DotNetSimCodec.enabled = true
	return {
		"sent": sent.finish().hex_encode().substr(0, 12), "payloads": payloads[0],
		"seen": seen.finish().hex_encode().substr(0, 12), "applied": applied, "failed": failed,
		"codecs": codecs, "ticks": ticks,
	}


## Script against codec on one player state, and on a client's whole 128-entity snapshot.
func _bench() -> void:
	print("bench (this machine, single thread)")
	const N := 128
	const LAPS := 30
	var writers: Array[Player] = []
	for i in N:
		var p := Player.new()
		p._bind(_config)
		p.position = Vector3(float(i), 2.0, -float(i) * 0.5)
		p.velocity = Vector3(1.5, 0.0, -2.0)
		p.yaw = float(i) * 2.7
		p.pitch = -10.0
		p.health = 75.0
		p.points = i * 10
		p.flags = i & 255
		p.spin = Quaternion(Vector3.UP, float(i) * 0.1)
		writers.append(p)
	# A typical moving player's state: position, velocity, yaw and pitch, spin.
	var picks := [0, 1, 2, 3, 12]
	var timings := {}
	for codec in [false, true]:
		var encode_us := 0
		var decode_us := 0
		var packets: Array = []
		var readers: Array[Player] = []
		for i in N:
			var r := Player.new()
			r._bind(_config)
			if not codec:
				r._sim_codec = null
			writers[i]._sim_codec = DotNetSimCodec.create(writers[i].net_vars, _config) if codec else null
			readers.append(r)
		for lap in LAPS:
			packets.clear()
			var at := Time.get_ticks_usec()
			for p in writers:
				var decls: Array[DotNetVar] = []
				for k: int in picks:
					decls.append(p.net_vars[k])
				packets.append(p._encode(decls, lap % 8)[0])
			encode_us += Time.get_ticks_usec() - at
			at = Time.get_ticks_usec()
			for i in N:
				var reader := DotNetReader.new(packets[i])
				reader.seek_bits(lap % 8)
				var _r := readers[i].read_state(reader, lap)
			decode_us += Time.get_ticks_usec() - at
		timings[codec] = [float(encode_us) / LAPS / 1000.0, float(decode_us) / LAPS / 1000.0]
	var s: Array = timings[false]
	var n: Array = timings[true]
	print("  encode %d states: script %.3f ms, codec %.3f ms (%.1fx)" % [N, s[0], n[0], s[0] / maxf(n[0], 0.0001)])
	print("  decode+apply %d states: script %.3f ms, codec %.3f ms (%.1fx)" % [N, s[1], n[1], s[1] / maxf(n[1], 0.0001)])
	print("BENCH %s" % JSON.stringify({"entities": N, "encode_script_ms": snappedf(s[0], 0.001), "encode_codec_ms": snappedf(n[0], 0.001),
		"apply_script_ms": snappedf(s[1], 0.001), "apply_codec_ms": snappedf(n[1], 0.001)}))
	_check(true, "bench ran")


## Bytes for the crate's own golden test (rust/crates/dot-sim-snapshot/src/tests.rs).
func _golden() -> void:
	var p := Player.new()
	p._bind(DotNetConfig.new())
	p._sim_codec = null
	p.position = Vector3(12.5, -3.25, 100.0)
	p.velocity = Vector3(4.0, 0.0, -2.5)
	p.yaw = 271.0
	p.pitch = -12.0
	p.health = 87.5
	p.stance = 5
	p.ammo = 300
	p.grounded = false
	p.spin = Quaternion(Vector3(0.0, 1.0, 0.0), 0.75)
	var decls: Array[DotNetVar] = []
	for k in [0, 1, 2, 3, 4, 5, 7, 10, 12]:
		decls.append(p.net_vars[k])
	var e := p._encode(decls, 3)
	print("GOLDEN pad 3, %d bits: %s" % [e[1], (e[0] as PackedByteArray).hex_encode()])
	_check(true, "golden printed")
