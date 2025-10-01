extern crate atomecs as lib;
extern crate nalgebra;
use bevy::prelude::*;
use lib::atom::{Position, Velocity};
use lib::atom_sources::emit::{AtomNumberToEmit, EmitFixedRate};
use lib::atom_sources::mass::{MassDistribution, MassRatio};
use lib::atom_sources::oven::{Oven, OvenAperture};
use lib::atom_sources::{AtomSourcePlugin, VelocityCap};
use lib::collisions::wall_collisions::{WallData, WallType};
use lib::collisions::{ApplyAtomCollisions, ApplyWallCollisions, CollisionPlugin};
use lib::constant::PI;
use lib::integrator::Timestep;
use lib::marker::{Interval, MarkerConfig, WriteOnce};
use lib::output::file::{FileOutputPlugin, Text};
use lib::probability_distribution::WeightedProbabilityDistribution;
use lib::shapes::{Cylinder as MyCylinder, CylindricalPipe};
use lib::sim_region::{SimulationVolume, VolumeType};
use lib::simulation::SimulationBuilder;
use lib::species::Strontium88;
use nalgebra::Vector3;
use std::marker::PhantomData;
use std::time::Instant;

fn main() {
    let now = Instant::now();

    let number_to_emit = 1e10;
    let radius = 25e-6;
    let length = 1000e-6;
    let direction = Vector3::new(1.0, 0.0, 0.0);
    let interval = 10;
    let timestep = 1e-7;

    let mut sim_builder = SimulationBuilder::default();
    sim_builder.add_plugins(AtomSourcePlugin::<Strontium88>::default());
    sim_builder.add_plugins(FileOutputPlugin::<Velocity, Text>::new(
        "vel.txt".to_string(),
        interval,
    ));
    sim_builder.add_plugins(CollisionPlugin);

    let mut sim = sim_builder.build();

    sim.world_mut().insert_resource(ApplyAtomCollisions(false));
    sim.world_mut().insert_resource(ApplyWallCollisions(true));
    sim.insert_resource(MarkerConfig {
        pos_range: vec![(0.0, f64::MAX), (f64::MIN, f64::MAX), (f64::MIN, f64::MAX)],
        ..Default::default()
    });
    sim.insert_resource(WriteOnce(true));
    sim.insert_resource(Interval(interval));

    sim.world_mut()
        .spawn(WallData {
            wall_type: WallType::Rough,
            wall_temp: Some(700.0),
            ..Default::default()
        })
        .insert(CylindricalPipe::new(radius, length, direction))
        .insert(Position {
            pos: direction * -length / 2.0,
        });

    sim.world_mut()
        .spawn(SimulationVolume {
            volume_type: VolumeType::Inclusive,
        })
        .insert(MyCylinder::new(1500e-6, 2000e-6, direction))
        .insert(Position {
            pos: Vector3::new(000e-6, 0.0, 0.0),
        });

    let mut thetas = Vec::<f64>::new();
    let mut weights = Vec::<f64>::new();

    let n = 1000;
    for i in 0..n {
        let theta = (i as f64) / (n as f64) * PI / 2.0;
        let weight = theta.sin() * theta.cos();
        thetas.push(theta);
        weights.push(weight);
    }

    let uniform_distribution = WeightedProbabilityDistribution::new(thetas, weights);

    sim.world_mut()
        .spawn(Oven::<Strontium88> {
            temperature: 700.0,
            aperture: OvenAperture::Circular {
                radius,
                thickness: 1e-9,
            },
            direction: direction,
            theta_distribution: uniform_distribution,
            max_theta: PI / 2.0,
            phantom: PhantomData,
        })
        .insert(Position {
            pos: Vector3::new(-length * 0.99999, 0.0, 0.0),
        })
        .insert(MassDistribution::new(vec![MassRatio {
            mass: 88.0,
            ratio: 1.0,
        }]))
        .insert(AtomNumberToEmit { number: 0 })
        .insert(EmitFixedRate {
            rate: number_to_emit,
        });

    // Define timestep
    sim.world_mut()
        .insert_resource(Timestep { delta: timestep });
    sim.world_mut()
        .insert_resource(VelocityCap { value: f64::MAX });

    // Run the simulation for a number of steps.
    for _i in 0..10_000 {
        sim.update();
    }
    println!("Simulation completed in {} ms.", now.elapsed().as_millis());
}
