//! Creation of atoms in a controlled manner and realease into the simulation

pub mod emit;
pub mod gaussian;
pub mod mass;
pub mod oven;
pub mod precalc;
pub mod species;
pub mod surface;

use self::species::AtomCreator;
use bevy::prelude::*;
use std::marker::PhantomData;

#[derive(Resource)]
pub struct VelocityCap {
    /// The maximum speed of an atom emitted by an atom source. See [Velocity](struct.Velocity.html) for units.
    pub value: f64,
}

impl Default for VelocityCap {
    fn default() -> Self {
        VelocityCap {
            value: std::f64::MAX,
        } // Default to no cap on velocity
    }
}

/// This plugin implements the creation of atoms of a given species from sources such as ovens or vacuum chambers.
///
/// See also [crate::atom_sources].
///
/// # Generic Arguments
///
/// * `T`: The atom species to create, which must implement the `AtomCreator` trait.
#[derive(Default)]
pub struct AtomSourcePlugin<T>(PhantomData<T>)
where
    T: AtomCreator;

impl<T> Plugin for AtomSourcePlugin<T>
where
    T: AtomCreator + 'static,
{
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            (
                emit::emit_number_per_frame_system,
                emit::emit_fixed_rate_system.after(emit::emit_number_per_frame_system),
                precalc::precalculate_for_species_system::<oven::Oven<T>>,
                precalc::precalculate_for_species_system::<surface::SurfaceSource<T>>,
                gaussian::precalculate_for_gaussian_source_system::<T>,
                oven::oven_create_atoms_system::<T>
                    .after(emit::emit_number_per_frame_system)
                    .after(precalc::precalculate_for_species_system::<oven::Oven<T>>),
                surface::create_atoms_on_surface_system::<T>
                    .after(emit::emit_number_per_frame_system)
                    .after(precalc::precalculate_for_species_system::<surface::SurfaceSource<T>>),
                gaussian::gaussian_create_atoms_system::<T>
                    .after(emit::emit_number_per_frame_system)
                    .after(gaussian::precalculate_for_gaussian_source_system::<T>),
                emit::emit_once_system
                    .after(oven::oven_create_atoms_system::<T>)
                    .after(surface::create_atoms_on_surface_system::<T>)
                    .after(gaussian::gaussian_create_atoms_system::<T>),
            ),
        );
    }
}
