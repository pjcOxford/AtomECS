use bevy::prelude::*;
use crate::atom::{Atom, Position, Velocity};
use crate::collisions::CollisionsSet;
use crate::initiate::NewlyCreated;

#[derive(PartialEq, Eq)]
pub enum WriteOrNot {
    Write,
    NotWrite,
}
#[derive(Component)]
pub struct Marker {
    pub write_status: WriteOrNot
}

pub fn init_marker_system (
    mut commands: Commands,
    query: Query<Entity, With<NewlyCreated>>
) {
    for entity in query.iter() {
        commands.entity(entity).insert(Marker {write_status: WriteOrNot::NotWrite});
    }
}

pub fn add_marker_system(
    mut query: Query<(Entity, &Position, &Velocity, &mut Marker)>,
) {
    for (entity, pos, vel, mut marker) in query.iter_mut() {
        // if pos.pos.x < 210e-6 && pos.pos.x > 200e-6 {
        //     commands.entity(entity).insert(Marker);
        // }
        // else if pos.pos.x > 210e-6 {
        //     commands.entity(entity).remove::<Marker>();
        // }
        if pos.pos.x > 0.0 {
            marker.write_status = WriteOrNot::Write;
        }
    }
}

pub struct MarkerPlugin;
impl Plugin for MarkerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(PreUpdate, init_marker_system);
        app.add_systems(PreUpdate, add_marker_system.after(CollisionsSet::WallCollisionSystems));
    }
}