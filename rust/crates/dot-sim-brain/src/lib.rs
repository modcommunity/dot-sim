//! The hot half of a dot-bot brain's think: [`aim`] (DotBotAim: view, aim error, reaction,
//! trigger) and [`fight`] (DotBotBrain's fight step: range, strafing, crouch, jump, the legs).
//!
//! Each is the script's arithmetic in the script's order and precision (engine vectors `f32`,
//! script floats `f64`, the script's random stream), so a brain thinking natively makes the same
//! decisions, to the bit, as the GDScript it replaces; the addon's parity suite holds it to that.

pub mod aim;
pub mod fight;
pub mod num;

pub use aim::{Aim, AimProfile, View};
pub use fight::{FightIn, FightOut, FightProfile, FirstShot, Strafe, fight};
pub use num::{Stream, Vec2, Vec3};
