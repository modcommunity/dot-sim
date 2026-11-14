//! The GDExtension: thin engine bindings over the pure crates, one cargo feature per module.

use godot::prelude::*;

#[cfg(feature = "arms")]
mod arms;
#[cfg(feature = "motor")]
mod batch;
#[cfg(feature = "bots")]
mod bots;
#[cfg(feature = "brain")]
mod brain;
#[cfg(feature = "motor")]
mod hulls;
#[cfg(feature = "interp")]
mod interp;
#[cfg(feature = "motor")]
mod motor;
#[cfg(feature = "snapshot")]
mod snapshot;

struct DotSimExtension;

#[gdextension]
unsafe impl ExtensionLibrary for DotSimExtension {}
