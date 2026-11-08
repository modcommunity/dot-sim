//! `DotSimBotSenses`: the sight scan of a bot's think, over one packed table per tick.

use dot_sim_bots::{Contacts, Found, Query, Vec3 as V3, candidates};
use godot::prelude::*;

/// The packed bodies of one tick and the last scan's answer.
#[derive(GodotClass)]
#[class(base = RefCounted, init)]
pub struct DotSimBotSenses {
    table: Contacts,
    found: Found,
}

#[godot_api]
impl DotSimBotSenses {
    /// Replaces the table: ids, aim points, teams and alive flags, one row per body.
    #[func]
    fn load_contacts(
        &mut self,
        ids: PackedInt64Array,
        points: PackedVector3Array,
        teams: PackedInt32Array,
        alive: PackedByteArray,
    ) {
        let n = ids
            .len()
            .min(points.len())
            .min(teams.len())
            .min(alive.len());
        let t = &mut self.table;
        t.ids.clear();
        t.points.clear();
        t.teams.clear();
        t.alive.clear();
        t.ids.extend_from_slice(&ids.as_slice()[..n]);
        t.points.extend(
            points.as_slice()[..n]
                .iter()
                .map(|p| V3::new(p.x, p.y, p.z)),
        );
        t.teams.extend_from_slice(&teams.as_slice()[..n]);
        t.alive
            .extend(alive.as_slice()[..n].iter().map(|a| *a != 0));
    }

    /// Indices of the bodies inside range and cone (or still tracked), in table order.
    #[func]
    #[allow(clippy::too_many_arguments)]
    fn candidates(
        &mut self,
        me_id: i64,
        me_team: i64,
        eye: Vector3,
        forward: Vector3,
        half_fov_deg: f64,
        sight: f64,
        close_notice: f64,
        target_id: i64,
        visible: bool,
    ) -> PackedInt32Array {
        let q = Query {
            me_id,
            me_team: me_team as i32,
            eye: V3::new(eye.x, eye.y, eye.z),
            forward: V3::new(forward.x, forward.y, forward.z),
            half_fov_deg,
            sight,
            close_notice,
            target_id,
            visible,
        };
        candidates(&self.table, &q, &mut self.found);
        PackedInt32Array::from(self.found.index.as_slice())
    }

    /// The last scan's scores, one per index it returned.
    #[func]
    fn scores(&self) -> PackedFloat32Array {
        PackedFloat32Array::from(self.found.scores.as_slice())
    }

    /// Whether the last scan found its visible target dead.
    #[func]
    fn target_dead(&self) -> bool {
        self.found.target_dead
    }
}
