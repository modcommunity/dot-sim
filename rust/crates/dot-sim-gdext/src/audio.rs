//! dot-audio's per-body probes, native: `DotSimSurfProbe` is `DotAudioSurfRules.is_surfing` (the ramp ray and
//! its per-physics-frame cache), `DotSimAudioSteps` runs `DotAudioFootsteps`' and `DotAudioSlide`'s tick over
//! the script's own body dictionary (see `dot-sim-audio` for the arithmetic, to the bit).
//!
//! The script keeps every body's state in its dictionary and plays every sound: a pass reads and writes the
//! same entries the script does and hands back what is left for the script (the steps to play, the slides
//! to advance), so a script `advance()` between passes and the script's own suites see one state.

use dot_sim_audio::{self as audio, SeenCache, SlideGate, StepState, StepTun, Vec3};
use godot::classes::{
    CharacterBody3D, CollisionObject3D, Engine, Node3D, PhysicsDirectSpaceState3D,
    PhysicsRayQueryParameters3D,
};
use godot::prelude::*;

#[inline]
fn v3(v: Vector3) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}

#[inline]
fn gv(v: Vec3) -> Vector3 {
    Vector3::new(v.x, v.y, v.z)
}

/// One reusable ray down from a body, excluding the body itself, as the scripts cast it.
struct DownRay {
    ray: Gd<PhysicsRayQueryParameters3D>,
    mask: i64,
    /// The body whose RID is excluded now (the scripts exclude the caster when it is a collision object).
    excluded: Option<InstanceId>,
}

impl DownRay {
    fn new() -> Self {
        let mut ray = PhysicsRayQueryParameters3D::new_gd();
        ray.set_collide_with_areas(false);
        ray.set_collision_mask(u32::MAX);
        Self {
            ray,
            mask: u32::MAX as i64,
            excluded: None,
        }
    }

    /// The hit's dictionary under `body` (empty for a miss or a body with no world).
    fn cast(
        &mut self,
        body: &Gd<Node3D>,
        pos: Vec3,
        depth: f64,
        mask: i64,
    ) -> Option<VarDictionary> {
        let world = body.get_world_3d()?;
        let mut space: Gd<PhysicsDirectSpaceState3D> = world.get_direct_space_state()?;
        let (from, to) = audio::ray_down(pos, depth);
        self.ray.set_from(gv(from));
        self.ray.set_to(gv(to));
        if mask != self.mask {
            self.ray.set_collision_mask(mask as u32);
            self.mask = mask;
        }
        let id = body.instance_id();
        if self.excluded != Some(id) {
            self.excluded = Some(id);
            let mut ex: Array<Rid> = Array::new();
            if let Ok(co) = body.clone().try_cast::<CollisionObject3D>() {
                ex.push(co.get_rid());
            }
            self.ray.set_exclude(&ex);
        }
        Some(space.intersect_ray(&self.ray))
    }
}

/// `DotAudioSurfRules.is_surfing`, native: a ray down `probe` metres, a ramp normal, one verdict per body per physics frame.
#[derive(GodotClass)]
#[class(base = RefCounted)]
pub struct DotSimSurfProbe {
    ray: DownRay,
    seen: SeenCache,
}

#[godot_api]
impl IRefCounted for DotSimSurfProbe {
    fn init(_base: Base<RefCounted>) -> Self {
        Self {
            ray: DownRay::new(),
            seen: SeenCache::default(),
        }
    }
}

impl DotSimSurfProbe {
    fn surfing(
        &mut self,
        body: &Gd<Node3D>,
        probe: f64,
        mask: i64,
        min_y: f64,
        max_y: f64,
    ) -> bool {
        let frame = Engine::singleton().get_physics_frames();
        let id = body.instance_id().to_i64();
        if let Some(yes) = self.seen.get(id, frame) {
            return yes;
        }
        let pos = v3(body.get_global_position());
        let yes = match self.ray.cast(body, pos, probe, mask) {
            Some(hit) if !hit.is_empty() => {
                let n = hit
                    .get("normal")
                    .and_then(|v| v.try_to::<Vector3>().ok())
                    .unwrap_or(Vector3::UP);
                audio::is_ramp(n.y, min_y, max_y)
            }
            _ => false,
        };
        self.seen.put(id, frame, yes);
        yes
    }
}

#[godot_api]
impl DotSimSurfProbe {
    /// Whether `body` is on a surf ramp now: `DotAudioSurfRules.is_surfing` with the rules' probe, mask and normal bounds.
    #[func]
    fn is_surfing(
        &mut self,
        body: Gd<Node3D>,
        probe: f64,
        mask: i64,
        min_normal_y: f64,
        max_normal_y: f64,
    ) -> bool {
        self.surfing(&body, probe, mask, min_normal_y, max_normal_y)
    }
}

/// The keys of the scripts' body entries (`String` keys, as GDScript's dictionary literals make them).
struct Keys {
    last: Variant,
    travelled: Variant,
    air: Variant,
    grounded: Variant,
    opts: Variant,
    on: Variant,
    sliding: Variant,
}

impl Keys {
    fn new() -> Self {
        let k = |s: &str| GString::from(s).to_variant();
        Self {
            last: k("last"),
            travelled: k("travelled"),
            air: k("air"),
            grounded: k("grounded"),
            opts: k("opts"),
            on: k("on"),
            sliding: k("sliding"),
        }
    }
}

/// `DotAudioFootsteps`' and `DotAudioSlide`'s tick over the script's body dictionary.
#[derive(GodotClass)]
#[class(base = RefCounted)]
pub struct DotSimAudioSteps {
    ray: DownRay,
    keys: Keys,
    no_args: VarArray,
}

#[godot_api]
impl IRefCounted for DotSimAudioSteps {
    fn init(_base: Base<RefCounted>) -> Self {
        Self {
            ray: DownRay::new(),
            keys: Keys::new(),
            no_args: VarArray::new(),
        }
    }
}

/// A live body in the tree, or `None` (freed, not a `Node3D`, or out of the tree).
fn live_body(key: &Variant) -> Option<Gd<Node3D>> {
    let body = key.try_to::<Gd<Node3D>>().ok()?;
    body.is_inside_tree().then_some(body)
}

#[godot_api]
impl DotSimAudioSteps {
    /// One `DotAudioFootsteps._physics_process` over `bodies` (its `_bodies`): drops the freed and the out-of-tree,
    /// advances the rest in place and returns the steps to play, `[body, loudness, ...]`, in the script's order.
    /// `tun` is `[quiet_speed, run_speed, stride_walk, stride_run, walk_loudness, landing_air_time]`;
    /// `hear_distance` 0 when there is no manager. `surf` is null (no surf muting), a `DotSimSurfProbe` asked with
    /// `surf_args` `[probe, mask, min_normal_y, max_normal_y]`, or a `Callable(body) -> bool`.
    #[func]
    #[allow(clippy::too_many_arguments)]
    fn steps_pass(
        &mut self,
        mut bodies: VarDictionary,
        delta: f64,
        tun: PackedFloat64Array,
        listener: Vector3,
        hear_distance: f64,
        surf: Variant,
        surf_args: PackedFloat64Array,
        ground_probe: f64,
        ground_mask: i64,
    ) -> VarArray {
        let mut out = VarArray::new();
        let tun = StepTun::from_slice(tun.as_slice());
        let probe = surf.try_to::<Gd<DotSimSurfProbe>>().ok();
        let callable = if probe.is_none() {
            surf.try_to::<Callable>().ok()
        } else {
            None
        };
        let sa = surf_args.as_slice();
        let sa = |i: usize| sa.get(i).copied().unwrap_or(0.0);
        let (s_probe, s_mask, s_min, s_max) = (sa(0), sa(1) as i64, sa(2), sa(3));
        let ear = v3(listener);
        let hear_sq = hear_distance * hear_distance;
        let k = &self.keys;
        let ray = &mut self.ray;
        let no_args = &self.no_args;
        for key in bodies.keys_array().iter_shared() {
            let Some(body) = live_body(&key) else {
                bodies.remove(&key);
                continue;
            };
            let Some(mut st) = bodies
                .get(&key)
                .and_then(|v| v.try_to::<VarDictionary>().ok())
            else {
                continue;
            };
            if st.is_empty() {
                continue;
            }
            let get_f = |st: &VarDictionary, key: &Variant| {
                st.get(key)
                    .and_then(|v| v.try_to::<f64>().ok())
                    .unwrap_or(0.0)
            };
            let before = StepState {
                last: st
                    .get(&k.last)
                    .and_then(|v| v.try_to::<Vector3>().ok())
                    .map(v3)
                    .unwrap_or(Vec3::new(0.0, 0.0, 0.0)),
                travelled: get_f(&st, &k.travelled),
                air: get_f(&st, &k.air),
                grounded: st.get(&k.grounded).is_some_and(|v| v.booleanize()),
            };
            let mut s = before;
            let pos = v3(body.get_global_position());
            let steps = audio::step_advance(
                &mut s,
                &tun,
                pos,
                delta,
                || hear_distance > 0.0 && (audio::distance_squared(ear, pos) as f64) > hear_sq,
                || match (&probe, &callable) {
                    (Some(p), _) => p
                        .clone()
                        .bind_mut()
                        .surfing(&body, s_probe, s_mask, s_min, s_max),
                    (None, Some(c)) => c.callv(&varray![&body.to_variant()]).booleanize(),
                    _ => false,
                },
                || {
                    let opts = st
                        .get(&k.opts)
                        .and_then(|v| v.try_to::<VarDictionary>().ok());
                    if let Some(g) = opts
                        .as_ref()
                        .and_then(|o| o.get("grounded"))
                        .and_then(|v| v.try_to::<Callable>().ok())
                    {
                        return g.callv(no_args).booleanize();
                    }
                    if let Ok(cb) = body.clone().try_cast::<CharacterBody3D>()
                        && cb.is_on_floor()
                    {
                        return true;
                    }
                    ray.cast(&body, pos, ground_probe, ground_mask)
                        .is_some_and(|hit| !hit.is_empty())
                },
            );
            st.set(&k.last, &gv(s.last).to_variant());
            if s.travelled.to_bits() != before.travelled.to_bits() {
                st.set(&k.travelled, &s.travelled.to_variant());
            }
            if s.air.to_bits() != before.air.to_bits() {
                st.set(&k.air, &s.air.to_variant());
            }
            if s.grounded != before.grounded {
                st.set(&k.grounded, &s.grounded.to_variant());
            }
            if steps.landing {
                out.push(&key);
                out.push(&1.0f64.to_variant());
            }
            if let Some(l) = steps.stride {
                out.push(&key);
                out.push(&l.to_variant());
            }
        }
        out
    }

    /// One `DotAudioSlide._physics_process` over `bodies` (its `_bodies`), as far as it needs no script: a body out of
    /// reach and not playing, or not playing and not worth asking, or asked (its `sliding` callable) and not sliding,
    /// only has its position kept. Returns `[body, code, ...]` for the rest, in order: 0 the script's own handling (a
    /// freed or out-of-tree body), 1 `out_of_reach`, 2 `advance`, 3 `advance` already asked and sliding.
    /// `quiet` is the speed a body not yet playing must reach to be asked (`DotAudioSurf`), negative to ask every body.
    #[func]
    fn slide_pass(
        &mut self,
        bodies: VarDictionary,
        delta: f64,
        reach_sq: f64,
        ear: Vector3,
        enabled: bool,
        quiet: f64,
    ) -> VarArray {
        let mut out = VarArray::new();
        let k = &self.keys;
        let ear = v3(ear);
        let quiet = (quiet >= 0.0).then_some(quiet);
        let send = |out: &mut VarArray, key: &Variant, code: i64| {
            out.push(key);
            out.push(&code.to_variant());
        };
        for key in bodies.keys_array().iter_shared() {
            let Some(body) = live_body(&key) else {
                send(&mut out, &key, 0);
                continue;
            };
            let Some(mut st) = bodies
                .get(&key)
                .and_then(|v| v.try_to::<VarDictionary>().ok())
            else {
                continue;
            };
            if st.is_empty() {
                continue;
            }
            let pos = v3(body.get_global_position());
            let on = st.get(&k.on).is_some_and(|v| v.booleanize());
            let last = st
                .get(&k.last)
                .and_then(|v| v.try_to::<Vector3>().ok())
                .map(v3)
                .unwrap_or(Vec3::new(0.0, 0.0, 0.0));
            let gate = audio::slide_gate(
                audio::distance_squared(pos, ear),
                reach_sq,
                on,
                enabled,
                || audio::slide_speed(last, pos, delta),
                quiet,
            );
            match gate {
                SlideGate::KeepFar | SlideGate::KeepQuiet => st.set(&k.last, &gv(pos).to_variant()),
                SlideGate::StopFar => send(&mut out, &key, 1),
                SlideGate::Advance if on => send(&mut out, &key, 2),
                SlideGate::Advance => {
                    let asked = st
                        .get(&k.sliding)
                        .and_then(|v| v.try_to::<Callable>().ok())
                        .map(|c| c.callv(&self.no_args).booleanize());
                    match asked {
                        Some(false) => st.set(&k.last, &gv(pos).to_variant()),
                        Some(true) => send(&mut out, &key, 3),
                        None => send(&mut out, &key, 2),
                    }
                }
            }
        }
        out
    }
}
