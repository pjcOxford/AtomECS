extern crate nalgebra;
extern crate atomecs as lib;

use rand::Rng;
use nalgebra::Vector3;
use bevy::prelude::*;
use lib::atom::{Atom, Position, Velocity, Force, Mass};
use lib::integrator::{Timestep, IntegrationPlugin, OldForce};
use lib::output::file::{FileOutputPlugin, Text};
use lib::marker::{Marker, MarkerPlugin, MarkerConfig, WriteOrNot};

fn main() {
    let mut rng = rand::rng();
    let mut app = App::new();

    app.add_plugins(FileOutputPlugin::<Position, Text>::new("pos.txt".to_string(), 1));
    app.add_plugins(MarkerPlugin);
    app.add_plugins(IntegrationPlugin);

    app.insert_resource(Timestep { delta: 1e-3 });
    // app.insert_resource(MarkerConfig {
    //     pos_range: vec![(0.5, 0.75), (f64::MIN, f64::MAX), (f64::MIN, f64::MAX)],
    //     vel_range: vec![(f64::MIN, f64::MAX), (f64::MIN, f64::MAX), (f64::MIN, f64::MAX)],
    // });

    for _ in 0..20 {
        app.world_mut().spawn((
            Position {
                pos: Vector3::new(
                    rng.random::<f64>(),
                    rng.random::<f64>(),
                    rng.random::<f64>(),
                ),
            },
            Velocity {vel: Vector3::new(1.0, 0.0, 0.0)},
            Force::default(),
            Mass {value: 1.0},
            OldForce(Force::default()),
            Marker {write_status: WriteOrNot::NotWrite},
            Atom,
        ));
    }
    for _i in 0..300 {
        app.update();
    }
}