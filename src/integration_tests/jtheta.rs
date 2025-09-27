
#[cfg(test)]
mod tests{
    use nalgebra::Vector3;
    use bevy::prelude::*;
    use std::marker::PhantomData;
    use crate::constant::PI;
    use crate::atom::{Atom, Position, Velocity};
    use crate::integrator::Timestep;
    use crate::simulation::SimulationBuilder;
    use crate::sim_region::{SimulationVolume, VolumeType};
    use crate::shapes::{Cylinder as MyCylinder, CylindricalPipe};
    use crate::species::Strontium88;
    use crate::atom_sources::{AtomSourcePlugin, WeightedProbabilityDistribution, VelocityCap};
    use crate::atom_sources::emit::{EmitFixedRate, AtomNumberToEmit};
    use crate::atom_sources::mass::{MassRatio, MassDistribution};
    use crate::atom_sources::oven::{OvenAperture, Oven};
    use crate::collisions::{CollisionPlugin, ApplyAtomCollisions, ApplyWallCollisions};
    use crate::collisions::wall_collisions::{WallData, WallType};
    use crate::marker::{MarkerConfig, WriteOrNot, Marker, WriteOnce, Interval};
    use crate::integrator::Step;

    fn analytic_jtheta(theta: f64, channel_length: f64, channel_radius: f64) -> f64 {
        let beta = 2.0 * channel_radius / channel_length; // (4.16)
        let q = theta.tan() / beta; // (4.19)
        let alpha = 0.5 // (4.16)
            - 1.0 / (3.0 * beta.powf(2.0))
                * (1.0 - 2.0 * beta.powf(3.0)
                    + (2.0 * beta.powf(2.0) - 1.0) * (1.0 + beta.powf(2.0)).powf(0.5))
                / ((1.0 + beta.powf(2.0)).powf(0.5) - beta.powf(2.0) * (1.0 / beta).asinh());

        let j_theta;
        if q <= 1.0 {
            let r_q = q.acos() - q * (1.0 - q.powf(2.0)).powf(0.5); // (4.23)
            j_theta = alpha * theta.cos()
                + (2.0 / PI)
                    * theta.cos() * ((1.0 - alpha) * r_q
                    + 2.0 / (3.0 * q) * (1.0 - 2.0 * alpha) * (1.0 - (1.0 - q.powf(2.0)).powf(1.5)))
        // (4.21)
        } else {
            j_theta = alpha * theta.cos() + 4.0 / (3.0 * PI * q) * (1.0 - 2.0 * alpha) * theta.cos();
            // (4.22)
        }
        j_theta
    }

    fn create_histogram_system(step: Res<Step>, mut query: Query<(&Velocity, &Marker)>, mut histogram: ResMut<Histogram>) {
        if step.n % 5 != 0 {
            return;
        }

        for (vel, m) in query.iter_mut() {

            if !(m.write_status == WriteOrNot::Write ){
                continue;
            }

            let theta = ((vel.vel[1].powi(2) + vel.vel[2].powi(2)).sqrt()/vel.vel[0]).atan();
            if theta >= 0.0 && theta < PI / 2.0 {
                let bin = (theta / (PI / 2.0) * 1000.0).floor() as usize;
                histogram.bins[bin] += 1;
            }
        }
    }

    fn compare_histogram_to_analytic(histogram: &Histogram, channel_length: f64, channel_radius: f64) {
        let total_counts: i32 = histogram.bins.iter().sum();
        let mut analytic_values = Vec::<f64>::new();
        let mut hist_values = Vec::<f64>::new();

        for i in 0..histogram.bins.len() {
            let theta = (i as f64 + 0.5) * (PI / 2.0) / (histogram.bins.len() as f64 + 1.0);
            analytic_values.push(analytic_jtheta(theta, channel_length, channel_radius) * theta.sin());
            hist_values.push(histogram.bins[i] as f64 / total_counts as f64);
        }

        // Normalize analytic values
        let analytic_sum: f64 = analytic_values.iter().sum();
        for val in analytic_values.iter_mut() {
            *val /= analytic_sum;
        }

        println!("Theta (rad), Analytic J(theta), Histogram J(theta)");
        for i in 0..histogram.bins.len() {
            let theta = (i as f64) * (PI / 2.0) / (histogram.bins.len() as f64);
            println!("{:.5}, {:.5e}, {:.5e}", theta, analytic_values[i], hist_values[i]);
        }

        // Print comparison
        let mean_square_error = analytic_values.iter().zip(hist_values.iter())
            .map(|(a, h)| (((a - h).abs()) / a.abs()).powi(2) / (histogram.bins.len() as f64))
            .fold(0.0, |acc, x| acc + x);
        assert!(mean_square_error < 0.01, "Max difference ratio {} exceeds 1%", mean_square_error);
        println!("Mean square error between analytic and histogram: {}", mean_square_error);
    }

    #[derive(Resource)]
    struct Histogram {
        bins: Vec<i32>,
    }

    impl Histogram {
        fn new(num_bins: usize) -> Self {
            Histogram {
                bins: vec![0; num_bins],
            }
        }
    }

    #[test]
    fn test_jtheta() {
        let mut sim_builder = SimulationBuilder::default();
        sim_builder.add_plugins(AtomSourcePlugin::<Strontium88>::default());
        sim_builder.add_plugins(CollisionPlugin);

        let mut sim = sim_builder.build();

        sim.world_mut().insert_resource(ApplyAtomCollisions(false));
        sim.world_mut().insert_resource(ApplyWallCollisions(true));
        sim.insert_resource(MarkerConfig {
            pos_range: vec![(0.0, f64::MAX), (f64::MIN, f64::MAX), (f64::MIN, f64::MAX)],
            vel_range: vec![(f64::MIN, f64::MAX), (f64::MIN, f64::MAX), (f64::MIN, f64::MAX)],
        });
        sim.insert_resource(WriteOnce(true));
        sim.insert_resource(Interval(5));
        sim.insert_resource(Histogram::new(1000 as usize));
        sim.add_systems(Update, create_histogram_system);
        
        let radius = 10e-6;
        let length = 400e-6;

        sim.world_mut()
            .spawn(WallData{
                wall_type: WallType::Rough})
            .insert(CylindricalPipe::new(radius, length, Vector3::new(1.0, 0.0, 0.0)))
            .insert(Position{pos: Vector3::new(-length / 2.0, 0.0, 0.0)});

        sim.world_mut()
            .spawn(SimulationVolume{volume_type: VolumeType::Inclusive})
            .insert(MyCylinder::new(1500e-6, 2000e-6, Vector3::new(1.0, 0.0, 0.0)))
            .insert(Position{pos: Vector3::new(600e-6, 0.0, 0.0)});

        let number_to_emit = 1e10;

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
                aperture: OvenAperture::Circular{radius, thickness: 1e-9},
                direction: Vector3::new(1.0, 0.0, 0.0),
                theta_distribution: uniform_distribution,
                max_theta: PI/2.0,
                phantom: PhantomData,
            })
            .insert(Position {
                pos: Vector3::new(- length * 0.99999, 0.0, 0.0),
            })
            .insert(MassDistribution::new(vec![MassRatio {
                mass: 88.0,
                ratio: 1.0,
            }]))
            .insert(AtomNumberToEmit{number: 0})
            .insert(EmitFixedRate{rate: number_to_emit});

        // Define timestep
        sim.world_mut().insert_resource(Timestep { delta: 2e-7 });
        sim.world_mut().insert_resource(VelocityCap {value: f64::MAX});

        // Run the simulation for a number of steps.
        for _i in 0..10000 {
            sim.update();
        }

        compare_histogram_to_analytic(&sim.world().get_resource::<Histogram>().unwrap(), length, radius);
    }
}
