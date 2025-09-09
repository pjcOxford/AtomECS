use bevy::prelude::*;
use std::marker::PhantomData;
use crate::atom::{Position, Velocity};
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

pub fn init_marker_system<C> (
    mut commands: Commands,
    query: Query<Entity, With<NewlyCreated>>
) where C: Component + Clone, 
{
    for entity in query.iter() {
        commands
            .entity(entity)
            .insert(Marker {
                write_status: WriteOrNot::NotWrite,
                marker: PhantomData::<C>
            });
    }
}

pub fn update_marker_system<C>(
    mut query: Query<(&Position, &Velocity, &mut Marker<C>)>,
    config: Res<MarkerConfig>,
    batch_strategy: Res<AtomECSBatchStrategy>,
) where C: Component + Clone 
{
    query
        .par_iter_mut()
        .batching_strategy(batch_strategy.0.clone())
        .for_each(|(pos, vel, mut marker)| {
            match marker.write_status {
                WriteOrNot::Write => {
                    let mut remove_marker = false;
                    for i in 0..3 {
                        if !(pos.pos[i] < config.pos_range[i].1 && pos.pos[i] > config.pos_range[i].0) {
                            remove_marker = true;
                        }
                        if !(vel.vel[i] < config.vel_range[i].1 && vel.vel[i] > config.vel_range[i].0) {
                            remove_marker = true;
                        }
                    }
                    if remove_marker {
                        marker.write_status = WriteOrNot::NotWrite;
                    }
                },
                WriteOrNot::NotWrite => {
                    let mut add_marker = true;
                    for i in 0..3 {
                        if !(pos.pos[i] < config.pos_range[i].1 && pos.pos[i] > config.pos_range[i].0) {
                            add_marker = false;
                        }
                        if !(vel.vel[i] < config.vel_range[i].1 && vel.vel[i] > config.vel_range[i].0) {
                            add_marker = false;
                        }
                    }
                    if add_marker {
                        marker.write_status = WriteOrNot::Write;
                    }
                },
                _ => ()
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
        app.add_systems(PreUpdate, init_marker_system::<Position>);
        app.add_systems(PreUpdate, init_marker_system::<Velocity>);
        app.add_systems(PreUpdate, update_marker_system::<Position>.after(CollisionsSet::WallCollisionSystems));
        app.add_systems(PreUpdate, update_marker_system::<Velocity>.after(CollisionsSet::WallCollisionSystems));
    }
}