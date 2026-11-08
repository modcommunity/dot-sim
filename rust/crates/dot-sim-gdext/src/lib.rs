//! The GDExtension: thin engine bindings over the pure crates, one cargo feature per module.

use godot::prelude::*;

#[cfg(feature = "bots")]
mod bots;

struct DotSimExtension;

#[gdextension]
unsafe impl ExtensionLibrary for DotSimExtension {}
