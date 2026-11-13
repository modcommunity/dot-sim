//! The GDExtension: thin engine bindings over the pure crates, one cargo feature per module.

use godot::prelude::*;

#[cfg(feature = "motor")]
mod batch;
#[cfg(feature = "bots")]
mod bots;
#[cfg(feature = "brain")]
mod brain;
#[cfg(feature = "figure")]
mod figure;
#[cfg(feature = "motor")]
mod hulls;
#[cfg(feature = "motor")]
mod motor;
#[cfg(feature = "snapshot")]
mod snapshot;

struct DotSimExtension;

#[gdextension]
unsafe impl ExtensionLibrary for DotSimExtension {}
