extern crate nalgebra;
extern crate atomecs as lib;

use criterion::{Criterion, criterion_group, criterion_main};
use rand::Rng;
use nalgebra::Vector3;
use bevy::prelude::*;
use lib::atom::{Atom, Position, Velocity};
use lib::integrator::{Timestep, IntegrationPlugin};
use lib::output::file::{FileOutputPlugin, Text};
use lib::marker::{Marker, MarkerPlugin};

criterion_group!(benches, bench_simulation);
criterion_main!{benches}

fn bench_simulation(c: &mut Criterion) {
    let mut group = c.benchmark_group("simulation");
    group.bench_function("run", |b| {
        b.iter(|| {
            let mut rng = rand::rng();
            let mut app = App::new();
            
            app.add_plugins(FileOutputPlugin::<Position, Text, Marker>::new("pos.txt".to_string(), 1));
            app.add_plugins(MarkerPlugin);
            app.add_plugins(IntegrationPlugin);

            app.insert_resource(Timestep { delta: 1e-7 });

            for _ in 0..100 {
                app.world_mut().spawn((
                    Position {
                        pos: Vector3::new(
                            rng.random::<f64>(),
                            rng.random::<f64>(),
                            rng.random::<f64>(),
                        ),
                    },
                    Velocity {
                        vel: Vector3::new(
                            rng.random::<f64>(),
                            rng.random::<f64>(),
                            rng.random::<f64>(),
                        ),
                    },
                    Atom,
                ));
            }
            for _i in 0..5 {
                app.update();
            }
        })
    });
    group.finish();
}
