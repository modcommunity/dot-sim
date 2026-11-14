class_name DotSim
extends RefCounted

## Which dot-sim modules are loaded. Nothing here needs the native library: on the web, or with
## no library installed, every answer is "absent" and callers keep their script path.

# No log channel: it only answers questions.

## Module name to the native class it registers.
const MODULES := {
	&"bots": &"DotSimBotSenses",
	&"snapshot": &"DotSimSnapshotCodec",
	&"interp": &"DotSimInterpolator",
	&"motor": &"DotSimFpsBody",
	&"arms": &"DotSimViewArms",
	&"audio": &"DotSimAudioSteps",
}


## Whether module [param module] (a key of [constant MODULES]) is loaded.
static func has(module: StringName) -> bool:
	var cls: StringName = MODULES.get(module, &"")
	return cls != &"" and ClassDB.class_exists(cls)


## The loaded modules' names.
static func loaded() -> Array[StringName]:
	var out: Array[StringName] = []
	for m: StringName in MODULES:
		if has(m):
			out.append(m)
	return out
