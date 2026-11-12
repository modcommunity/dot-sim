//! Where an engine query comes from, for a per-call-site profile (`DotSimFpsBody.set_profile`).
//!
//! The motor marks the stage it is in ([`stage`], restored when the guard drops) and the body the
//! step of a sweep or rest it is at ([`step`]); the engine side reads [`current`] when it times a
//! query. One thread-local byte pair: nothing is read unless a profile runs, and nothing here
//! changes an answer.

use std::cell::Cell;

thread_local! {
    static CUR_STAGE: Cell<u8> = const { Cell::new(0) };
    static CUR_STEP: Cell<u8> = const { Cell::new(0) };
}

/// Motor stages, in tick order.
pub const STAGES: [&str; 13] = [
    "other",
    "depenetrate",
    "ground_start",
    "wall_contact",
    "wall_move",
    "slide",
    "step",
    "ground_move",
    "snap",
    "ground_end",
    "quadrant",
    "edge",
    "lift",
];
pub const OTHER: u8 = 0;
pub const DEPENETRATE: u8 = 1;
pub const GROUND_START: u8 = 2;
pub const WALL_CONTACT: u8 = 3;
pub const WALL_MOVE: u8 = 4;
pub const SLIDE: u8 = 5;
pub const STEP: u8 = 6;
pub const GROUND_MOVE: u8 = 7;
pub const SNAP: u8 = 8;
pub const GROUND_END: u8 = 9;
pub const QUADRANT: u8 = 10;
pub const EDGE: u8 = 11;
pub const LIFT: u8 = 12;

/// Steps of one body query.
pub const STEPS: [&str; 11] = [
    "none",
    "pass_out_overlap",
    "cast",
    "contact_rest",
    "retry_rest",
    "start_rest",
    "end_rest",
    "flip_rest",
    "ray",
    "overlaps",
    "rest_contact",
];
pub const PASS_OUT_OVERLAP: u8 = 1;
pub const CAST: u8 = 2;
pub const CONTACT_REST: u8 = 3;
pub const RETRY_REST: u8 = 4;
pub const START_REST: u8 = 5;
pub const END_REST: u8 = 6;
pub const FLIP_REST: u8 = 7;
pub const RAY: u8 = 8;
pub const OVERLAPS: u8 = 9;
pub const REST_CONTACT: u8 = 10;

/// Restores the stage it replaced when dropped.
pub struct Stage(u8);

impl Drop for Stage {
    #[inline]
    fn drop(&mut self) {
        CUR_STAGE.with(|c| c.set(self.0));
    }
}

/// Enters a stage until the guard drops.
#[inline]
#[must_use]
pub fn stage(s: u8) -> Stage {
    Stage(CUR_STAGE.with(|c| c.replace(s)))
}

/// The step a body query is at.
#[inline]
pub fn step(s: u8) {
    CUR_STEP.with(|c| c.set(s));
}

/// The stage and the step now.
#[inline]
pub fn current() -> (u8, u8) {
    (CUR_STAGE.with(Cell::get), CUR_STEP.with(Cell::get))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_stage_guard_restores_the_outer_stage() {
        let _a = stage(SLIDE);
        {
            let _b = stage(QUADRANT);
            step(CAST);
            assert_eq!(current(), (QUADRANT, CAST));
        }
        assert_eq!(current().0, SLIDE);
    }
}
