use bevy::prelude::*;
use crate::atom::{Atom, Position, Velocity};
use crate::collisions::CollisionsSet;

#[derive(Component)]
pub struct Marker;

#[derive(Resource)]
pub struct MarkerConfig {
    pub pos_range: Vec<(f64, f64)>,
    pub vel_range: Vec<(f64, f64)>,
}

pub fn add_marker_system(
    mut commands: Commands,
    query: Query<(Entity, &Position, &Velocity), With<Atom>>,
    config: Res<MarkerConfig>,
) {
    for (entity, pos, vel) in query.iter() {
        let mut add_marker = true;
        for i in 0..3 {
            if !(pos.pos.x < config.pos_range[i].1 && pos.pos.x > config.pos_range[i].0) {
                add_marker = false;
            }
            if !(vel.vel.x < config.vel_range[i].1 && vel.vel.x > config.vel_range[i].0) {
                add_marker = false;
            }
        }
        if add_marker {
            commands.entity(entity).insert(Marker);
        }
    }
}

pub fn remove_marker_system(
    mut commands: Commands,
    query: Query<(Entity, &Position, &Velocity), With<Atom>>,
    config: Res<MarkerConfig>,
) {
    for (entity, pos, vel) in query.iter() {
        let mut remove_marker = false;
        for i in 0..3 {
            if !(pos.pos.x < config.pos_range[i].1 && pos.pos.x > config.pos_range[i].0) {
                remove_marker = true;
            }
            if !(vel.vel.x < config.vel_range[i].1 && vel.vel.x > config.vel_range[i].0) {
                remove_marker = true;
            }
        }
        if remove_marker {
            commands.entity(entity).remove::<Marker>();
        }
    }
}

pub struct MarkerPlugin;
impl Plugin for MarkerPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(MarkerConfig {
            pos_range: vec![(f64::MIN, f64::MAX), (f64::MIN, f64::MAX), (f64::MIN, f64::MAX)],
            vel_range: vec![(f64::MIN, f64::MAX), (f64::MIN, f64::MAX), (f64::MIN, f64::MAX)],
        });
        app.add_systems(PreUpdate, add_marker_system.after(CollisionsSet::WallCollisionSystems));
        app.add_systems(PreUpdate, remove_marker_system.after(CollisionsSet::WallCollisionSystems));
    }
}