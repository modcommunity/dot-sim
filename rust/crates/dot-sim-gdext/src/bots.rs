//! `DotSimBotSenses`: the sight scan of a bot's think, over one packed table per tick.

use dot_sim_bots::{Bodies, Contacts, Found, Query, Vec3 as V3, candidates};
use godot::prelude::*;

/// The packed bodies of one tick and the last scan's answer.
#[derive(GodotClass)]
#[class(base = RefCounted, init)]
pub struct DotSimBotSenses {
    pub(crate) table: Contacts,
    /// The bodies' eyes, row for row with `table` once [`Self::load_eyes`] gave them.
    pub(crate) eyes: Vec<V3>,
    found: Found,
    /// The columns [`Self::load_bodies`] was given, kept for the near test.
    bodies: Bodies,
    near: Vec<u8>,
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
        self.eyes.clear();
        self.bodies.clear();
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

    /// The bodies' eyes, in the rows [`Self::load_contacts`] was given; the brain's native perceive
    /// tests sight to them.
    #[func]
    fn load_eyes(&mut self, eyes: PackedVector3Array) {
        self.eyes.clear();
        self.eyes
            .extend(eyes.as_slice().iter().map(|p| V3::new(p.x, p.y, p.z)));
    }

    /// Replaces the table from the game's own columns, one row per body: ids, feet, teams, alive
    /// flags, eye and aim heights. Derives the aim points and eyes here, so the script neither
    /// builds them nor reads a contact object for them; replaces [`Self::load_contacts`] plus
    /// [`Self::load_eyes`].
    #[func]
    fn load_bodies(
        &mut self,
        ids: PackedInt64Array,
        positions: PackedVector3Array,
        teams: PackedInt32Array,
        alive: PackedByteArray,
        eye_heights: PackedFloat32Array,
        aim_heights: PackedFloat32Array,
    ) {
        let n = ids
            .len()
            .min(positions.len())
            .min(teams.len())
            .min(alive.len())
            .min(eye_heights.len())
            .min(aim_heights.len());
        let b = &mut self.bodies;
        b.clear();
        b.ids.extend_from_slice(&ids.as_slice()[..n]);
        b.positions.extend(
            positions.as_slice()[..n]
                .iter()
                .map(|p| V3::new(p.x, p.y, p.z)),
        );
        b.teams.extend_from_slice(&teams.as_slice()[..n]);
        b.alive
            .extend(alive.as_slice()[..n].iter().map(|a| *a != 0));
        b.eye_heights
            .extend_from_slice(&eye_heights.as_slice()[..n]);
        b.aim_heights
            .extend_from_slice(&aim_heights.as_slice()[..n]);
        b.fill(&mut self.table, &mut self.eyes);
    }

    /// One byte per row of [`Self::load_bodies`]: 1 when no live enemy of that body stands closer
    /// than `near` metres, feet to feet (the game's think-rate test, for every body at once).
    #[func]
    fn nobody_near(&mut self, near: f64) -> PackedByteArray {
        self.bodies.nobody_near_all(near, &mut self.near);
        PackedByteArray::from(self.near.as_slice())
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
