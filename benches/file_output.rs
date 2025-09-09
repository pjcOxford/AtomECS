extern crate nalgebra;
extern crate atomecs as lib;

use criterion::{Criterion, criterion_group, criterion_main, black_box};
use rand::Rng;
use nalgebra::Vector3;
use bevy::prelude::*;
use lib::atom::{Atom, Position, Velocity};
use lib::integrator::{Timestep, IntegrationPlugin};
use lib::output::file::{FileOutputPlugin, Text};
use lib::marker::{Marker, MarkerPlugin, MarkerConfig};

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
            app.insert_resource(MarkerConfig {
                pos_range: vec![(0.5, 0.75), (f64::MIN, f64::MAX), (f64::MIN, f64::MAX)],
                vel_range: vec![(f64::MIN, f64::MAX), (f64::MIN, f64::MAX), (f64::MIN, f64::MAX)],
            });

            for _ in 0..800 {
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
                            black_box(1.0),    
                            black_box(0.0),
                            black_box(0.0),
                        ),
                    },
                    Atom,
                ));
            }
            for _i in 0..300 {
                app.update();
            }
        })
    });
    group.finish();
}
