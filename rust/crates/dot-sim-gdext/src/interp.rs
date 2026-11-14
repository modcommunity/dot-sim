//! `DotSimInterpolator`: the tracks and the per-frame draw of dot-net's `DotNetInterpolator`,
//! writing what the script's `apply` writes, value for value.

use std::collections::HashMap;

use dot_sim_interp::{Config, Counts, Interp, Other, Value, Values};
use godot::prelude::*;

/// A value the crate leaves opaque. A quaternion slerps and a colour lerps in the engine itself,
/// so they match the script's to the bit; everything else switches at the midpoint.
#[derive(Clone)]
struct Var(Variant);

impl Other for Var {
    fn same_kind(&self, other: &Self) -> bool {
        self.0.get_type() == other.0.get_type()
    }

    fn blend(&self, to: &Self, alpha: f64) -> Option<Self> {
        let method = match self.0.get_type() {
            VariantType::QUATERNION => "slerp",
            VariantType::COLOR => "lerp",
            _ => return None,
        };
        Some(Var(self
            .0
            .call(method, &[to.0.clone(), alpha.to_variant()])))
    }
}

/// One behaviour's part of a track's write plan, as the script's `_build_plan` makes it.
struct Part {
    id: InstanceId,
    size: usize,
    /// Interpolated properties in declaration order: interned key, name.
    props: Vec<(u32, StringName)>,
    /// Whether writes go through `_net_write_property` (overridden) rather than `set`.
    hook: bool,
}

#[derive(Default)]
struct Plan(Vec<Part>);

#[derive(GodotClass)]
#[class(base = RefCounted, init)]
pub struct DotSimInterpolator {
    interp: Interp<Var, Plan>,
    keys: HashMap<StringName, u32>,
    names: Vec<StringName>,
    /// Script instance id -> whether it overrides `_net_write_property`.
    hooks: HashMap<i64, bool>,
}

fn to_value(v: &Variant) -> Value<Var> {
    match v.get_type() {
        VariantType::FLOAT => Value::Float(v.to::<f64>()),
        VariantType::VECTOR3 => {
            let p = v.to::<Vector3>();
            Value::Vec3([p.x, p.y, p.z])
        }
        VariantType::VECTOR2 => {
            let p = v.to::<Vector2>();
            Value::Vec2([p.x, p.y])
        }
        _ => Value::Other(Var(v.clone())),
    }
}

fn to_variant(v: &Value<Var>) -> Variant {
    match v {
        Value::Float(f) => f.to_variant(),
        Value::Vec3(p) => Vector3::new(p[0], p[1], p[2]).to_variant(),
        Value::Vec2(p) => Vector2::new(p[0], p[1]).to_variant(),
        Value::Other(o) => o.0.clone(),
    }
}

fn net_vars(behaviour: &Gd<Object>) -> Option<AnyArray> {
    behaviour.get("net_vars").try_to::<AnyArray>().ok()
}

impl DotSimInterpolator {
    fn key(&mut self, name: StringName) -> u32 {
        if let Some(k) = self.keys.get(&name) {
            return *k;
        }
        let k = self.names.len() as u32;
        self.names.push(name.clone());
        self.keys.insert(name, k);
        k
    }

    /// Whether `behaviour`'s script overrides `_net_write_property`: the method listed more
    /// than once, the script's own over its base's. A script that is not a GDScript is asked.
    fn hooked(&mut self, behaviour: &Gd<Object>) -> bool {
        let Some(script) = behaviour.get_script() else {
            return false;
        };
        let id = script.instance_id().to_i64();
        if let Some(h) = self.hooks.get(&id) {
            return *h;
        }
        let hook = !script.is_class("GDScript")
            || script
                .get_script_method_list()
                .iter_shared()
                .filter(|m| {
                    m.get("name")
                        .is_some_and(|n| n.to_string() == "_net_write_property")
                })
                .count()
                > 1;
        self.hooks.insert(id, hook);
        hook
    }

    fn build_plan(&mut self, behaviours: &[Gd<Object>]) -> Plan {
        let mut parts = Vec::with_capacity(behaviours.len());
        for behaviour in behaviours {
            let vars = net_vars(behaviour);
            let mut props = Vec::new();
            if let Some(vars) = &vars {
                for declaration in vars.iter_shared() {
                    let Ok(declaration) = declaration.try_to::<Gd<Object>>() else {
                        continue;
                    };
                    if declaration.get("interpolate").booleanize() {
                        let name = declaration.get("property").to::<StringName>();
                        props.push((self.key(name.clone()), name));
                    }
                }
            }
            parts.push(Part {
                id: behaviour.instance_id(),
                size: vars.map_or(0, |v| v.len()),
                props,
                hook: self.hooked(behaviour),
            });
        }
        Plan(parts)
    }
}

fn plan_fits(plan: &Plan, behaviours: &[Gd<Object>]) -> bool {
    plan.0.len() == behaviours.len()
        && plan.0.iter().zip(behaviours).all(|(part, b)| {
            part.id == b.instance_id() && net_vars(b).map_or(0, |v| v.len()) == part.size
        })
}

#[godot_api]
impl DotSimInterpolator {
    /// `DotNetInterpolator.push`'s track half: a snapshot's values for an entity.
    #[func]
    fn push(&mut self, net_id: i64, tick: i64, values: VarDictionary) {
        let mut out: Values<Var> = Vec::with_capacity(values.len());
        for (k, v) in values.iter_shared() {
            // A dictionary finds a `String` key by the `StringName` the script looks up with.
            let name = match k.get_type() {
                VariantType::STRING_NAME => k.to::<StringName>(),
                VariantType::STRING => StringName::from(&k.to::<GString>()),
                _ => continue,
            };
            out.push((self.key(name), to_value(&v)));
        }
        self.interp.push(net_id, tick, out);
    }

    /// `DotNetInterpolator.apply` past its predicted check: samples at `render` (the script's
    /// `server_tick - delay_ticks()`), eases a resume, and writes each behaviour's interpolated
    /// properties, then calls its `_net_interpolated(int(server_tick))`. Config is passed per
    /// call, read as the script reads it. Returns whether anything was sampled.
    #[allow(clippy::too_many_arguments)]
    #[func]
    fn apply(
        &mut self,
        net_id: i64,
        behaviours: AnyArray,
        server_tick: f64,
        render: f64,
        frame_elapsed: f64,
        max_ticks: f64,
        ease_max: f64,
        ease_sec: f64,
    ) -> bool {
        let cfg = Config {
            max_ticks,
            ease_max,
            ease_sec,
        };
        let Some(values) = self.interp.draw(net_id, render, frame_elapsed, &cfg) else {
            return false;
        };
        let list: Vec<Gd<Object>> = behaviours
            .iter_shared()
            .filter_map(|b| b.try_to::<Gd<Object>>().ok())
            .collect();
        // Out of the track while hooks run, back in after.
        let mut plan = self
            .interp
            .plan_mut(net_id)
            .map(std::mem::take)
            .unwrap_or_default();
        if !plan_fits(&plan, &list) {
            plan = self.build_plan(&list);
        }
        let tick = (server_tick as i64).to_variant();
        for (part, behaviour) in plan.0.iter().zip(list.iter()) {
            let mut behaviour = behaviour.clone();
            let mut wrote = false;
            for (key, name) in &part.props {
                let Some((_, value)) = values.iter().find(|(k, _)| k == key) else {
                    continue;
                };
                let value = to_variant(value);
                if part.hook {
                    behaviour.call("_net_write_property", &[name.to_variant(), value]);
                } else {
                    behaviour.set(name, &value);
                }
                wrote = true;
            }
            if wrote {
                behaviour.call("_net_interpolated", std::slice::from_ref(&tick));
            }
        }
        if let Some(slot) = self.interp.plan_mut(net_id) {
            *slot = plan;
        }
        true
    }

    /// `DotNetInterpolator.sample` at `render`, counted as it counts: for a suite.
    #[func]
    fn sample(&mut self, net_id: i64, render: f64, max_ticks: f64) -> VarDictionary {
        let cfg = Config {
            max_ticks,
            ease_max: 0.0,
            ease_sec: 1.0,
        };
        let mut out = VarDictionary::new();
        for (k, v) in self.interp.sample(net_id, render, &cfg) {
            out.set(&self.names[k as usize], &to_variant(&v));
        }
        out
    }

    #[func]
    fn forget(&mut self, net_id: i64) {
        self.interp.forget(net_id);
    }

    #[func]
    fn clear(&mut self) {
        self.interp.clear();
    }

    #[func]
    fn prune(&mut self, live_ids: PackedInt64Array) {
        self.interp.prune(live_ids.as_slice());
    }

    #[func]
    fn track_count(&self) -> i64 {
        self.interp.track_count() as i64
    }

    #[func]
    fn sample_count(&self) -> i64 {
        self.interp.counts.samples
    }

    #[func]
    fn stall_count(&self) -> i64 {
        self.interp.counts.stalls
    }

    #[func]
    fn extrapolation_count(&self) -> i64 {
        self.interp.counts.extrapolations
    }

    #[func]
    fn reset_counts(&mut self) {
        self.interp.counts = Counts::default();
    }
}
