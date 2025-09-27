use bevy::prelude::*;
use crate::atom::{Position, Velocity, Atom};
use crate::collisions::CollisionsSet;
use crate::integrator::{Step, AtomECSBatchStrategy};

#[derive(PartialEq, Eq)]
pub enum WriteOrNot {
    Write,
    NotWrite,
    NeverWrite
}

#[derive(Component)]
pub struct Marker {
    pub write_status: WriteOrNot,
}

#[derive(Resource)]
pub struct MarkerConfig {
    pub pos_range: Vec<(f64, f64)>,
    pub vel_range: Vec<(f64, f64)>,
}

impl Default for MarkerConfig {
    fn default() -> Self {
        MarkerConfig {
            pos_range: vec![(f64::MIN, f64::MAX), (f64::MIN, f64::MAX), (f64::MIN, f64::MAX)],
            vel_range: vec![(f64::MIN, f64::MAX), (f64::MIN, f64::MAX), (f64::MIN, f64::MAX)],
        }
    }
}

/// A resource to signify whether atom need to be written to file only once or multiple times.
/// Assumes that atoms need to be written on first appearance within the marker region.
#[derive(Resource)]
pub struct WriteOnce(pub bool);

#[derive(Resource)]
pub struct Interval(pub u64);

fn is_in_range(pos: &Position, vel: &Velocity, config: &MarkerConfig) -> bool {
    for i in 0..3 {
        if !(pos.pos[i] < config.pos_range[i].1 && pos.pos[i] > config.pos_range[i].0) || 
            !(vel.vel[i] < config.vel_range[i].1 && vel.vel[i] > config.vel_range[i].0) {
            return false;
        }
    }
    true
}

fn update_marker_system(
    mut commands: Commands,
    mut query_new: Query<(Entity, &Position, &Velocity), (With<Atom>, Without<Marker>)>,
    mut query_existing: Query<(&Position, &Velocity, &mut Marker)>,
    config: Res<MarkerConfig>,
    batch_strategy: Res<AtomECSBatchStrategy>,
) {
    query_existing
        .par_iter_mut()
        .batching_strategy(batch_strategy.0.clone())
        .for_each(|(pos, vel, mut marker)| {
            match marker.write_status {
                WriteOrNot::Write => {
                    if !is_in_range(pos, vel, &config) {marker.write_status = WriteOrNot::NotWrite}
                },
                WriteOrNot::NotWrite => {
                    if is_in_range(pos, vel, &config) {marker.write_status = WriteOrNot::Write}
                },
                WriteOrNot::NeverWrite => ()
            }
    });

    query_new.iter_mut()
        .for_each(|(entity, pos, vel)| {
            if is_in_range(pos, vel, &config) {
                commands.entity(entity)
                    .insert(Marker {write_status: WriteOrNot::Write});
            }
    });
}

// System to set atoms that have already been written once to never write again.
// Works only for one common interval for all file outputs.
fn dont_write_to_written_once_atoms_system (
    mut query: Query<&mut Marker>,
    write_once: Res<WriteOnce>,
    step: Res<Step>,
    interval: Res<Interval>,
) {
    if !write_once.0 {return;}
    if step.n % interval.0 != 0 {return;} 
    query.iter_mut()
        .for_each(|mut marker| {
            if marker.write_status == WriteOrNot::Write {
                marker.write_status = WriteOrNot::NeverWrite;
            }
    });
}


pub struct MarkerPlugin;
impl Plugin for MarkerPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(MarkerConfig::default());
        app.insert_resource(WriteOnce(false));
        app.insert_resource(Interval(u64::MAX));
        app.add_systems(PreUpdate, update_marker_system.after(CollisionsSet::WallCollisionSystems));
        app.add_systems(PostUpdate, dont_write_to_written_once_atoms_system);
    }
}