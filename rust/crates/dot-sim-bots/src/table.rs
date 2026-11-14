//! The bodies of one tick as columns: the game writes them once from its own state, and every
//! kernel of the tick reads them (the sight scan's table, the eyes, the near test) without going
//! back to script objects. Reading an engine object's fields from native code costs about three
//! times what the script pays, so the gain only comes when the data never lives in such fields.

use crate::{Contacts, Vec3, is_enemy};

/// One row per body: id, feet, team, alive, and the heights of its eye and aim point above the feet.
#[derive(Default, Debug, Clone)]
pub struct Bodies {
    pub ids: Vec<i64>,
    pub positions: Vec<Vec3>,
    pub teams: Vec<i32>,
    pub alive: Vec<bool>,
    pub eye_heights: Vec<f32>,
    pub aim_heights: Vec<f32>,
}

impl Bodies {
    /// Number of bodies.
    pub fn len(&self) -> usize {
        self.ids.len()
    }

    /// Whether there are none.
    pub fn is_empty(&self) -> bool {
        self.ids.is_empty()
    }

    /// Empties every column, keeping their memory.
    pub fn clear(&mut self) {
        self.ids.clear();
        self.positions.clear();
        self.teams.clear();
        self.alive.clear();
        self.eye_heights.clear();
        self.aim_heights.clear();
    }

    /// The sight scan's table and the eyes, as `DotBotWorld._pack` builds them from the contacts:
    /// `aim_point()` and `eye()` are the feet plus `Vector3.UP` times the height, in `f32`.
    pub fn fill(&self, out: &mut Contacts, eyes: &mut Vec<Vec3>) {
        out.ids.clear();
        out.points.clear();
        out.teams.clear();
        out.alive.clear();
        eyes.clear();
        out.ids.extend_from_slice(&self.ids);
        out.teams.extend_from_slice(&self.teams);
        out.alive.extend_from_slice(&self.alive);
        for i in 0..self.len() {
            let p = self.positions[i];
            out.points.push(p + up(self.aim_heights[i]));
            eyes.push(p + up(self.eye_heights[i]));
        }
    }

    /// Whether no live enemy of row `me` stands closer than `near` metres (feet to feet), as the
    /// game's `_nobody_near` asks before letting a bot think less often.
    pub fn nobody_near(&self, me: usize, near: f64) -> bool {
        let near_sq = near * near;
        let (me_id, me_team, at) = (self.ids[me], self.teams[me], self.positions[me]);
        for i in 0..self.len() {
            if self.alive[i]
                && is_enemy(me_id, me_team, self.ids[i], self.teams[i])
                && ((self.positions[i] - at).length_squared() as f64) < near_sq
            {
                return false;
            }
        }
        true
    }

    /// [`Self::nobody_near`] for every row, one flag each (1: nobody near).
    pub fn nobody_near_all(&self, near: f64, out: &mut Vec<u8>) {
        out.clear();
        out.extend((0..self.len()).map(|i| u8::from(self.nobody_near(i, near))));
    }
}

/// `Vector3.UP * h`: the engine multiplies each component, so x and z are `0 * h`.
#[inline]
fn up(h: f32) -> Vec3 {
    Vec3::new(0.0 * h, h, 0.0 * h)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bodies() -> Bodies {
        let mut b = Bodies::default();
        let rows = [
            (1, Vec3::new(0.0, 0.0, 0.0), 1, true),
            (2, Vec3::new(0.0, 0.0, -29.0), 2, true), // an enemy inside 30 m
            (3, Vec3::new(5.0, 0.0, 0.0), 1, true),   // a friend, close
            (4, Vec3::new(100.0, 0.0, 0.0), 2, true), // far from everyone
            (5, Vec3::new(1.0, 0.0, 1.0), 2, false),  // a dead enemy, close
            (6, Vec3::new(0.0, 0.0, 30.0), 0, true),  // team 0, exactly 30 m from 1
        ];
        for (id, p, team, alive) in rows {
            b.ids.push(id);
            b.positions.push(p);
            b.teams.push(team);
            b.alive.push(alive);
            b.eye_heights.push(1.6);
            b.aim_heights.push(1.25);
        }
        b
    }

    #[test]
    fn fill_lifts_points_and_eyes() {
        let b = bodies();
        let mut c = Contacts::default();
        let mut eyes = Vec::new();
        b.fill(&mut c, &mut eyes);
        assert_eq!(c.len(), 6);
        assert_eq!(c.points[1], Vec3::new(0.0, 1.25, -29.0));
        assert_eq!(eyes[3], Vec3::new(100.0, 1.6, 0.0));
        assert!(!c.alive[4]);
    }

    #[test]
    fn near_ignores_friends_the_dead_and_the_edge() {
        let b = bodies();
        assert!(!b.nobody_near(0, 30.0)); // 2 is 29 m away
        assert!(b.nobody_near(0, 29.0)); // strictly closer; 6 at 30 m does not count either
        assert!(b.nobody_near(3, 30.0)); // 4 is alone
        let mut out = Vec::new();
        b.nobody_near_all(30.0, &mut out);
        assert_eq!(out, vec![0, 0, 0, 1, 0, 1]);
    }
}
