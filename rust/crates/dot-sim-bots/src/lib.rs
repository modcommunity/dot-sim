//! Bot brain kernels: pure Rust over plain arrays, no engine types.
//!
//! The first kernel is the sight scan of a bot's think: which bodies are inside its range and
//! view cone, with their closeness score. It is the brain's `_candidates`, the same arithmetic in
//! the same precision, so the brain's choices do not change with the kernel.

pub use dot_sim_math::Vec3;
use dot_sim_math::rad_to_deg;

/// Beyond this many degrees off the view, a body already being fought is no longer tracked.
pub const TRACK_DEG: f64 = 90.0;

/// Degrees off the view that double a body's score (its distance).
pub const SCORE_DEG: f64 = 60.0;

/// The bodies of one tick, packed once for every bot that thinks in it.
#[derive(Default, Debug, Clone)]
pub struct Contacts {
    pub ids: Vec<i64>,
    pub points: Vec<Vec3>,
    pub teams: Vec<i32>,
    pub alive: Vec<bool>,
}

impl Contacts {
    /// Number of bodies.
    pub fn len(&self) -> usize {
        self.ids.len()
    }

    /// Whether there are none.
    pub fn is_empty(&self) -> bool {
        self.ids.is_empty()
    }
}

/// One bot's question: who it is, where it looks from and along, and how far and wide it sees.
#[derive(Debug, Clone, Copy)]
pub struct Query {
    pub me_id: i64,
    pub me_team: i32,
    pub eye: Vec3,
    pub forward: Vec3,
    pub half_fov_deg: f64,
    pub sight: f64,
    pub close_notice: f64,
    pub target_id: i64,
    pub visible: bool,
}

/// The scan's answer: candidate indices into [`Contacts`] in order, their scores, and whether
/// the visible target was found dead (the brain then forgets it).
#[derive(Default, Debug, Clone)]
pub struct Found {
    pub index: Vec<i32>,
    pub scores: Vec<f32>,
    pub target_dead: bool,
}

/// Fills `out` with the bodies `q` could be looking at, as the script's `_candidates` does.
pub fn candidates(c: &Contacts, q: &Query, out: &mut Found) {
    out.index.clear();
    out.scores.clear();
    out.target_dead = false;
    let mut visible = q.visible;
    for i in 0..c.len() {
        let id = c.ids[i];
        if !c.alive[i] && id == q.target_id && visible {
            out.target_dead = true;
            visible = false;
            continue;
        }
        if !c.alive[i] || !is_enemy(q.me_id, q.me_team, id, c.teams[i]) {
            continue;
        }
        let to = c.points[i] - q.eye;
        let d = to.length() as f64;
        if d > q.sight {
            continue;
        }
        let off = if d > 0.01 {
            rad_to_deg(q.forward.angle_to(to) as f64)
        } else {
            0.0
        };
        let tracking = visible && id == q.target_id && off < TRACK_DEG;
        if off > q.half_fov_deg && d > q.close_notice && !tracking {
            continue;
        }
        out.index.push(i as i32);
        out.scores.push((d * (1.0 + off / SCORE_DEG)) as f32);
    }
}

/// Whether body `id` of `team` is somebody `me_id` of `me_team` fights; team 0 fights everybody.
#[inline]
pub fn is_enemy(me_id: i64, me_team: i32, id: i64, team: i32) -> bool {
    id != me_id && (me_team == 0 || team == 0 || me_team != team)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn world() -> Contacts {
        let mut c = Contacts::default();
        let rows = [
            (1, Vec3::new(0.0, 1.25, 0.0), 1, true),   // the bot itself
            (2, Vec3::new(0.0, 1.25, -10.0), 2, true), // straight ahead
            (3, Vec3::new(10.0, 1.25, -1.0), 2, true), // 84 degrees off
            (4, Vec3::new(0.0, 1.25, -5.0), 1, true),  // a friend
            (5, Vec3::new(1.0, 1.25, 1.0), 2, true),   // behind, but touching
            (6, Vec3::new(0.0, 1.25, -90.0), 2, true), // out of sight range
            (7, Vec3::new(0.5, 1.25, -8.0), 2, false), // dead
        ];
        for (id, p, team, alive) in rows {
            c.ids.push(id);
            c.points.push(p);
            c.teams.push(team);
            c.alive.push(alive);
        }
        c
    }

    fn query() -> Query {
        Query {
            me_id: 1,
            me_team: 1,
            eye: Vec3::new(0.0, 1.6, 0.0),
            forward: Vec3::new(0.0, 0.0, -1.0),
            half_fov_deg: 50.0,
            sight: 60.0,
            close_notice: 2.0,
            target_id: -1,
            visible: false,
        }
    }

    #[test]
    fn scan_keeps_enemies_in_cone_and_close_ones() {
        let mut f = Found::default();
        candidates(&world(), &query(), &mut f);
        assert_eq!(f.index, vec![1, 4]);
        assert!(!f.target_dead);
        assert!(f.scores[0] > 10.0 && f.scores[0] < 10.5);
    }

    #[test]
    fn a_visible_target_is_tracked_outside_the_cone_below_ninety_degrees() {
        let mut q = query();
        q.target_id = 3;
        q.visible = true;
        let mut f = Found::default();
        candidates(&world(), &q, &mut f);
        assert!(f.index.contains(&2), "{:?}", f.index);
    }

    #[test]
    fn a_dead_visible_target_is_reported_and_skipped() {
        let mut q = query();
        q.target_id = 7;
        q.visible = true;
        let mut f = Found::default();
        candidates(&world(), &q, &mut f);
        assert!(f.target_dead);
        assert!(!f.index.contains(&6));
    }

    #[test]
    fn team_zero_fights_everybody_but_itself() {
        assert!(is_enemy(1, 0, 2, 1));
        assert!(is_enemy(1, 1, 2, 0));
        assert!(!is_enemy(1, 1, 2, 1));
        assert!(!is_enemy(1, 0, 1, 0));
    }
}
