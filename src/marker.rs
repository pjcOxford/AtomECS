use bevy::prelude::*;
use std::marker::PhantomData;
use crate::atom::{Position, Velocity, Atom};
use crate::collisions::CollisionsSet;
use crate::initiate::NewlyCreated;
use crate::integrator::AtomECSBatchStrategy;

#[derive(PartialEq, Eq)]
pub enum WriteOrNot {
    Write,
    NotWrite,
    NeverWrite
}
#[derive(Component)]
pub struct Marker<C: Component> {
    pub write_status: WriteOrNot,
    pub marker: PhantomData<C>,
}

#[derive(Resource)]
pub struct MarkerConfig {
    pub pos_range: Vec<(f64, f64)>,
    pub vel_range: Vec<(f64, f64)>,
}

fn is_within_bounds(pos: &Position, vel: &Velocity, config: &MarkerConfig) -> bool {
    for i in 0..3 {
        if !(pos.pos[i] < config.pos_range[i].1 && pos.pos[i] > config.pos_range[i].0) || 
            !(vel.vel[i] < config.vel_range[i].1 && vel.vel[i] > config.vel_range[i].0) {
            return false;
        }
    }
    true
}

pub fn update_marker_system<C>(
    mut commands: Commands,
    mut query_new: Query<(Entity, &Position, &Velocity), (With<Atom>, Without<Marker<C>>)>,
    mut query_existing: Query<(&Position, &Velocity, &mut Marker<C>)>,
    config: Res<MarkerConfig>,
    batch_strategy: Res<AtomECSBatchStrategy>,
) where C: Component + Clone 
{
    query_existing
        .par_iter_mut()
        .batching_strategy(batch_strategy.0.clone())
        .for_each(|(pos, vel, mut marker)| {
            match marker.write_status {
                WriteOrNot::Write => {
                    if !is_within_bounds(pos, vel, &config) {
                        marker.write_status = WriteOrNot::NotWrite;
                    }
                },
                WriteOrNot::NotWrite => {
                    if is_within_bounds(pos, vel, &config) {
                        marker.write_status = WriteOrNot::Write;
                    }
                },
                WriteOrNot::NeverWrite => ()
            }
    });

    query_new.iter_mut()
        .for_each(|(entity, pos, vel)| {
            if is_within_bounds(pos, vel, &config) {
                commands.entity(entity)
                    .insert(Marker {
                        write_status: WriteOrNot::Write,
                        marker: PhantomData::<C>
                });
            }
    });
}


pub struct MarkerPlugin;
impl Plugin for MarkerPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(MarkerConfig {
            pos_range: vec![(f64::MIN, f64::MAX), (f64::MIN, f64::MAX), (f64::MIN, f64::MAX)],
            vel_range: vec![(f64::MIN, f64::MAX), (f64::MIN, f64::MAX), (f64::MIN, f64::MAX)],
        });
        app.add_systems(PreUpdate, update_marker_system::<Position>.after(CollisionsSet::WallCollisionSystems));
        app.add_systems(PreUpdate, update_marker_system::<Velocity>.after(CollisionsSet::WallCollisionSystems));
    }
}