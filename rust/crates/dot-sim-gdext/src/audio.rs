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

    /// The hit's dictionary under `body` (empty for a miss or a body with no world). `rid` is the body's own RID
    /// when it is a collision object, asked only when the excluded body changes.
    fn cast(
        &mut self,
        body: &Gd<Node3D>,
        rid: impl FnOnce() -> Option<Rid>,
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
            if let Some(r) = rid() {
                ex.push(r);
            }
            self.ray.set_exclude(&ex);
        }
        Some(space.intersect_ray(&self.ray))
    }
}

/// A body's own RID when it is a collision object (the scripts exclude the caster then).
fn rid_of(body: &Gd<Node3D>) -> Option<Rid> {
    body.clone()
        .try_cast::<CollisionObject3D>()
        .ok()
        .map(|co| co.get_rid())
}

/// `DotAudioSurfRules.is_surfing`, native: a ray down `probe` metres, a ramp normal, one verdict per body per physics frame.
#[derive(GodotClass)]
#[class(base = RefCounted)]
pub struct DotSimSurfProbe {
    ray: DownRay,
    seen: SeenCache,
    normal: Variant,
}

#[godot_api]
impl IRefCounted for DotSimSurfProbe {
    fn init(_base: Base<RefCounted>) -> Self {
        Self {
            ray: DownRay::new(),
            seen: SeenCache::default(),
            normal: GString::from("normal").to_variant(),
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
        self.surfing_at(
            body,
            || rid_of(body),
            id,
            pos,
            frame,
            probe,
            mask,
            min_y,
            max_y,
        )
    }

    /// [`Self::surfing`] for a caller that already knows the body's id, position and the physics frame.
    #[allow(clippy::too_many_arguments)]
    fn surfing_at(
        &mut self,
        body: &Gd<Node3D>,
        rid: impl FnOnce() -> Option<Rid>,
        id: i64,
        pos: Vec3,
        frame: u64,
        probe: f64,
        mask: i64,
        min_y: f64,
        max_y: f64,
    ) -> bool {
        if let Some(yes) = self.seen.get(id, frame) {
            return yes;
        }
        let yes = match self.ray.cast(body, rid, pos, probe, mask) {
            Some(hit) if !hit.is_empty() => {
                let n = hit
                    .get(&self.normal)
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

/// `DotAudioFootsteps`' and `DotAudioSlide`'s tick: the bodies' state is held here, loaded from the scripts' dictionaries.
#[derive(GodotClass)]
#[class(base = RefCounted)]
pub struct DotSimAudioSteps {
    ray: DownRay,
    no_args: VarArray,
    /// The packed footstep pass's bodies, in the script's dictionary order (see `steps_load`).
    steps: Vec<StepBody>,
    /// The packed slide pass's bodies, in the script's dictionary order (see `slide_load`).
    slides: Vec<SlideBody>,
}

/// One body of the packed slide pass. `node` is None once the body is dropped (the script reloads then).
struct SlideBody {
    node: Option<Gd<Node3D>>,
    rid: Option<Rid>,
    last: Vec3,
    on: bool,
    sliding: Option<Callable>,
    /// Its `sliding` is the stock `DotAudioSurf._is_surfer`: asked of the surf probe here.
    surfer: bool,
}

/// The values per answer of `slide_tick`: index, code, previous last x, y, z.
const SLIDE_STRIDE: usize = 5;
/// One body of the packed footstep pass: its key in the script's dictionary, the node and what it is, its state.
struct StepBody {
    key: Variant,
    node: Option<Gd<Node3D>>,
    cb: Option<Gd<CharacterBody3D>>,
    rid: Option<Rid>,
    grounded: Option<Callable>,
    st: StepState,
}

/// The values per body of `steps_load` and `steps_state`: last x, y, z, travelled, air, grounded (0 or 1).
const STEP_STRIDE: usize = 6;

#[godot_api]
impl IRefCounted for DotSimAudioSteps {
    fn init(_base: Base<RefCounted>) -> Self {
        Self {
            ray: DownRay::new(),
            no_args: VarArray::new(),
            steps: Vec::new(),
            slides: Vec::new(),
        }
    }
}

#[godot_api]
impl DotSimAudioSteps {
    /// Loads the packed footstep pass: `keys` the script's body keys in order, `state` their states (`STEP_STRIDE`
    /// values each: last x, y, z, travelled, air, grounded) and `grounded` each body's `grounded` callable or null.
    /// The state is held here from now on; `steps_state` reads it back.
    #[func]
    fn steps_load(&mut self, keys: VarArray, state: PackedFloat64Array, grounded: VarArray) {
        let st = state.as_slice();
        self.steps.clear();
        for (i, key) in keys.iter_shared().enumerate() {
            let node = key.try_to::<Gd<Node3D>>().ok();
            let cb = node
                .as_ref()
                .and_then(|n| n.clone().try_cast::<CharacterBody3D>().ok());
            let rid = node.as_ref().and_then(rid_of);
            let at = |k: usize| st.get(i * STEP_STRIDE + k).copied().unwrap_or(0.0);
            self.steps.push(StepBody {
                key,
                node,
                cb,
                rid,
                grounded: grounded.get(i).and_then(|v| v.try_to::<Callable>().ok()),
                st: StepState {
                    last: Vec3::new(at(0) as f32, at(1) as f32, at(2) as f32),
                    travelled: at(3),
                    air: at(4),
                    grounded: at(5) != 0.0,
                },
            });
        }
    }

    /// The packed pass's states, `STEP_STRIDE` values per body in the loaded order (dropped bodies gone).
    #[func]
    fn steps_state(&self) -> PackedFloat64Array {
        let mut out = Vec::with_capacity(self.steps.len() * STEP_STRIDE);
        for b in &self.steps {
            let s = &b.st;
            out.extend_from_slice(&[
                s.last.x as f64,
                s.last.y as f64,
                s.last.z as f64,
                s.travelled,
                s.air,
                if s.grounded { 1.0 } else { 0.0 },
            ]);
        }
        PackedFloat64Array::from(out.as_slice())
    }

    /// One `DotAudioFootsteps._physics_process` over the loaded bodies: returns `[key, loudness, ...]` in order, -1 for a body
    /// dropped (freed or out of the tree) that the script erases.
    /// `tun` is `[quiet_speed, run_speed, stride_walk, stride_run, walk_loudness, landing_air_time]`;
    /// `hear_distance` 0 when there is no manager. `surf` is null (no surf muting), a `DotSimSurfProbe` asked with
    /// `surf_args` `[probe, mask, min_normal_y, max_normal_y]`, or a `Callable(body) -> bool`.
    #[func]
    #[allow(clippy::too_many_arguments)]
    fn steps_tick(
        &mut self,
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
        let mut probe = surf.try_to::<Gd<DotSimSurfProbe>>().ok();
        let callable = if probe.is_none() {
            surf.try_to::<Callable>().ok()
        } else {
            None
        };
        let mut probe = probe.as_mut().map(|p| p.bind_mut());
        let sa = surf_args.as_slice();
        let sa = |i: usize| sa.get(i).copied().unwrap_or(0.0);
        let (s_probe, s_mask, s_min, s_max) = (sa(0), sa(1) as i64, sa(2), sa(3));
        let ear = v3(listener);
        let hear_sq = hear_distance * hear_distance;
        let mut frame: Option<u64> = None;
        let ray = &mut self.ray;
        let no_args = &self.no_args;
        let mut i = 0;
        while i < self.steps.len() {
            let b = &mut self.steps[i];
            let live = b
                .node
                .as_ref()
                .is_some_and(|n| n.is_instance_valid() && n.is_inside_tree());
            if !live {
                out.push(&b.key);
                out.push(&(-1.0f64).to_variant());
                self.steps.remove(i);
                continue;
            }
            i += 1;
            let Some(node) = b.node.as_ref() else {
                continue;
            };
            let pos = v3(node.get_global_position());
            let rid = b.rid;
            let steps = audio::step_advance(
                &mut b.st,
                &tun,
                pos,
                delta,
                || hear_distance > 0.0 && (audio::distance_squared(ear, pos) as f64) > hear_sq,
                || match (&mut probe, &callable) {
                    (Some(p), _) => {
                        let f =
                            *frame.get_or_insert_with(|| Engine::singleton().get_physics_frames());
                        p.surfing_at(
                            node,
                            || rid,
                            node.instance_id().to_i64(),
                            pos,
                            f,
                            s_probe,
                            s_mask,
                            s_min,
                            s_max,
                        )
                    }
                    (None, Some(c)) => c.callv(&varray![&node.to_variant()]).booleanize(),
                    _ => false,
                },
                || {
                    if let Some(g) = &b.grounded {
                        return g.callv(no_args).booleanize();
                    }
                    if b.cb.as_ref().is_some_and(|cb| cb.is_on_floor()) {
                        return true;
                    }
                    ray.cast(node, || rid, pos, ground_probe, ground_mask)
                        .is_some_and(|hit| !hit.is_empty())
                },
            );
            if steps.landing {
                out.push(&b.key);
                out.push(&1.0f64.to_variant());
            }
            if let Some(l) = steps.stride {
                out.push(&b.key);
                out.push(&l.to_variant());
            }
        }
        out
    }

    /// Loads the packed slide pass: `bodies` in the script's order, their `last` positions and `on` flags, their
    /// `sliding` callables and whether each is the stock surfer question (`surfer`, 0 or 1).
    #[func]
    fn slide_load(
        &mut self,
        bodies: VarArray,
        last: PackedVector3Array,
        on: PackedByteArray,
        sliding: VarArray,
        surfer: PackedByteArray,
    ) {
        let (last, on, surfer) = (last.as_slice(), on.as_slice(), surfer.as_slice());
        self.slides.clear();
        for (i, key) in bodies.iter_shared().enumerate() {
            let node = key.try_to::<Gd<Node3D>>().ok();
            let rid = node.as_ref().and_then(rid_of);
            self.slides.push(SlideBody {
                node,
                rid,
                last: last
                    .get(i)
                    .copied()
                    .map(v3)
                    .unwrap_or(Vec3::new(0.0, 0.0, 0.0)),
                on: on.get(i).is_some_and(|&b| b != 0),
                sliding: sliding.get(i).and_then(|v| v.try_to::<Callable>().ok()),
                surfer: surfer.get(i).is_some_and(|&b| b != 0),
            });
        }
    }

    /// The `on` flags after the script handled `slide_tick`'s answers: each value `index * 2 + on`.
    #[func]
    fn slide_on(&mut self, on: PackedInt32Array) {
        for &v in on.as_slice() {
            if let Some(b) = self.slides.get_mut((v / 2) as usize) {
                b.on = v % 2 == 1;
            }
        }
    }

    /// Every loaded body's `last` position, in the loaded order.
    #[func]
    fn slide_last(&self) -> PackedVector3Array {
        let v: Vec<Vector3> = self.slides.iter().map(|b| gv(b.last)).collect();
        PackedVector3Array::from(v.as_slice())
    }

    /// One `DotAudioSlide._physics_process` gate over the loaded bodies (`reach_sq` the tick's `reach_squared()`,
    /// `quiet` the speed a body not yet playing must reach to be asked, negative: every body is asked). Every body's `last` becomes its position (as every script path does).
    /// Returns `SLIDE_STRIDE` values per body the script must handle: index, code, the `last` before this tick.
    /// Codes: 0 freed or out of the tree (dropped here), 1 `out_of_reach`, 2 `_advance` (not asked), 3 `_advance`
    /// asked and sliding, 4 `_stop` (playing, asked or switched off, and not sliding). `surf` is the surf probe for
    /// the stock surfer bodies (null: their callables are asked), `surf_args` its arguments, `surf_on` the rules'
    /// `enabled`.
    #[func]
    #[allow(clippy::too_many_arguments)]
    fn slide_tick(
        &mut self,
        delta: f64,
        reach_sq: f64,
        ear: Vector3,
        enabled: bool,
        quiet: f64,
        surf: Variant,
        surf_args: PackedFloat64Array,
        surf_on: bool,
    ) -> PackedFloat64Array {
        let mut out: Vec<f64> = Vec::with_capacity(self.slides.len() * SLIDE_STRIDE);
        let ear = v3(ear);
        let quiet = (quiet >= 0.0).then_some(quiet);
        let mut probe = surf.try_to::<Gd<DotSimSurfProbe>>().ok();
        let mut probe = probe.as_mut().map(|p| p.bind_mut());
        let sa = surf_args.as_slice();
        let sa = |i: usize| sa.get(i).copied().unwrap_or(0.0);
        let (s_probe, s_mask, s_min, s_max) = (sa(0), sa(1) as i64, sa(2), sa(3));
        let mut frame: Option<u64> = None;
        let no_args = &self.no_args;
        for (i, b) in self.slides.iter_mut().enumerate() {
            let Some(node) = b.node.as_ref() else {
                continue;
            };
            let mut send = |code: f64, last: Vec3| {
                out.extend_from_slice(&[
                    i as f64,
                    code,
                    last.x as f64,
                    last.y as f64,
                    last.z as f64,
                ]);
            };
            if !(node.is_instance_valid() && node.is_inside_tree()) {
                b.node = None;
                send(0.0, b.last);
                continue;
            }
            let pos = v3(node.get_global_position());
            let last = b.last;
            b.last = pos;
            let gate = audio::slide_gate(
                audio::distance_squared(pos, ear),
                reach_sq,
                b.on,
                enabled,
                || audio::slide_speed(last, pos, delta),
                quiet,
            );
            match gate {
                SlideGate::KeepFar | SlideGate::KeepQuiet => {}
                SlideGate::StopFar => send(1.0, last),
                SlideGate::Advance if b.on && !enabled => send(4.0, last),
                SlideGate::Advance => {
                    let asked = match (&mut probe, b.surfer) {
                        (Some(p), true) => Some(
                            surf_on && {
                                let f = *frame.get_or_insert_with(|| {
                                    Engine::singleton().get_physics_frames()
                                });
                                let rid = b.rid;
                                p.surfing_at(
                                    node,
                                    || rid,
                                    node.instance_id().to_i64(),
                                    pos,
                                    f,
                                    s_probe,
                                    s_mask,
                                    s_min,
                                    s_max,
                                )
                            },
                        ),
                        _ => b.sliding.as_ref().map(|c| c.callv(no_args).booleanize()),
                    };
                    match asked {
                        Some(true) => send(3.0, last),
                        Some(false) if b.on => send(4.0, last),
                        Some(false) => {}
                        None => send(2.0, last),
                    }
                }
            }
        }
        PackedFloat64Array::from(out.as_slice())
    }
}
