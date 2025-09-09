extern crate nalgebra;
extern crate atomecs as lib;

use criterion::{Criterion, criterion_group, criterion_main};
use std::hint::black_box;
use rand::Rng;
use nalgebra::Vector3;
use bevy::prelude::*;
use lib::atom::{Atom, Position, Velocity, Force, Mass};
use lib::integrator::{Timestep, IntegrationPlugin, OldForce};
use lib::output::file::{FileOutputPlugin, Text};
use lib::marker::{Marker, MarkerPlugin, MarkerConfig, WriteOrNot};

criterion_group!(benches, output_bench);
criterion_main!{benches}

fn output_bench(c: &mut Criterion) {
    let mut group = c.benchmark_group("file output");
    
    group.bench_function("add + remove", |b| {
        b.iter(|| {
            let mut rng = rand::rng();
            let mut app = App::new();
            
            app.add_plugins(FileOutputPlugin::<Position, Text>::new("pos.txt".to_string(), 1));
            app.add_plugins(MarkerPlugin);
            app.add_plugins(IntegrationPlugin);

            app.insert_resource(Timestep { delta: 1e-2 });
            app.insert_resource(MarkerConfig {
                pos_range: vec![(0.5, 0.6), (f64::MIN, f64::MAX), (f64::MIN, f64::MAX)],
                vel_range: vec![(f64::MIN, f64::MAX), (f64::MIN, f64::MAX), (f64::MIN, f64::MAX)],
            });

            for _ in 0..20_000 {
                app.world_mut().spawn((
                    Position {
                        pos: Vector3::new(
                            rng.random::<f64>(),
                            black_box(0.0),
                            black_box(0.0),
                        ),
                    },
                    Velocity {
                        vel: Vector3::new(
                            black_box(1.0),    
                            black_box(0.0),
                            black_box(0.0),
                        ),
                    },
                    Atom,
                    Force::default(),
                    Mass {value: 1.0},
                    OldForce(Force::default()),
                    Marker {write_status: WriteOrNot::NotWrite},
                ));
            }
            for _i in 0..100 {
                app.update();
            }
        })
    });

    group.bench_function("add, no remove", |b| {
        b.iter(|| {
            let mut rng = rand::rng();
            let mut app = App::new();
            
            app.add_plugins(FileOutputPlugin::<Position, Text>::new("pos.txt".to_string(), 1));
            app.add_plugins(MarkerPlugin);
            app.add_plugins(IntegrationPlugin);

            app.insert_resource(Timestep { delta: 1e-10 });
            app.insert_resource(MarkerConfig {
                pos_range: vec![(0.5, 0.6), (f64::MIN, f64::MAX), (f64::MIN, f64::MAX)],
                vel_range: vec![(f64::MIN, f64::MAX), (f64::MIN, f64::MAX), (f64::MIN, f64::MAX)],
            });

            for _ in 0..20_000 {
                app.world_mut().spawn((
                    Position {
                        pos: Vector3::new(
                            rng.random::<f64>(),
                            black_box(0.0),
                            black_box(0.0),
                        ),
                    },
                    Velocity {
                        vel: Vector3::new(
                            black_box(1.0),    
                            black_box(0.0),
                            black_box(0.0),
                        ),
                    },
                    Atom,
                    Force::default(),
                    Mass {value: 1.0},
                    OldForce(Force::default()),
                    Marker {write_status: WriteOrNot::NotWrite},
                ));
            }
            for _i in 0..100 {
                app.update();
            }
        })
    });
    group.finish();
}
