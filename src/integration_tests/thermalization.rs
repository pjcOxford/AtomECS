#[cfg(test)]
mod tests {
    use crate::atom::{Atom, Force, Mass, Position, Velocity};
    use crate::collisions::atom_collisions::{CollisionParameters, CrossSection};
    use crate::collisions::wall_collisions::{WallData, WallType};
    use crate::collisions::CollisionPlugin;
    use crate::constant::{AMU, BOLTZCONST, EXP, PI};
    use crate::initiate::NewlyCreated;
    use crate::integrator::Step;
    use crate::integrator::Timestep;
    use crate::marker::{Marker, WriteOrNot};
    use crate::shapes::Sphere as MySphere;
    use crate::sim_region::{SimulationVolume, VolumeType};
    use crate::simulation::SimulationBuilder;
    use bevy::prelude::*;
    use nalgebra::Vector3;
    use rand_distr::{Distribution, Uniform};

    const TEMP: f64 = 0.002; // Wall temperature in Kelvin
    const MASS: f64 = 87.0 * AMU; // Mass of atom in amu

    fn maxwellian(velocity: f64) -> f64 {
        let coeff = 4.0 * PI * velocity.powi(2) * (MASS / (2.0 * PI * BOLTZCONST * TEMP)).powf(1.5);
        let exponent = -MASS * velocity.powi(2) / (2.0 * BOLTZCONST * TEMP);
        coeff * EXP.powf(exponent)
    }

    fn create_histogram_system(
        step: Res<Step>,
        mut query: Query<(&Velocity, &Marker)>,
        mut histogram: ResMut<Histogram>,
    ) {
        if step.n != 1999 {
            return;
        }

        for (vel, m) in query.iter_mut() {
            if !(m.write_status == WriteOrNot::Write) {
                continue;
            }
            if vel.vel.norm() > 0.0 && vel.vel.norm() < 3.0 {
                let bin = (vel.vel.norm() / 3.0 * 100.0).floor() as usize;
                histogram.bins[bin] += 1;
            }
        }
    }

    fn compare_histogram_to_analytic(histogram: &Histogram) {
        let total_counts: i32 = histogram.bins.iter().sum();
        let mut analytic_values = Vec::<f64>::new();
        let mut hist_values = Vec::<f64>::new();

        for i in 0..histogram.bins.len() {
            let velocity = (i as f64) * (3.0) / (histogram.bins.len() as f64);
            analytic_values.push(maxwellian(velocity));
            hist_values.push(histogram.bins[i] as f64 / total_counts as f64);
        }

        // Normalize analytic values
        let analytic_sum: f64 = analytic_values.iter().sum();
        for val in analytic_values.iter_mut() {
            *val /= analytic_sum;
        }

        let mean_square_error = analytic_values
            .iter()
            .zip(hist_values.iter())
            .map(|(a, h)| ((a - h).abs()).powi(2) / (histogram.bins.len() as f64))
            .fold(0.0, |acc, x| acc + x);
        for i in 0..100 {
            print!(" {} ", i as f64 / 100.0 * 3.0);
            print!(" {} ", analytic_values[i]);
            println!(" {} ", hist_values[i]);
        }
        println!("mse: {}", mean_square_error);
        assert!(mean_square_error < 5e-6,);
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
    fn test_wall_temp() {
        let mut sim_builder = SimulationBuilder::default();
        sim_builder.add_plugins(CollisionPlugin);

        let mut sim = sim_builder.build();

        let p_dist = Uniform::new(-25e-3, 25e-3).unwrap();
        let v_dist = Uniform::new(-5e-1, 5e-1).unwrap();
        for _i in 0..50_000 {
            sim.world_mut()
                .spawn(Position {
                    pos: Vector3::new(
                        p_dist.sample(&mut rand::rng()),
                        p_dist.sample(&mut rand::rng()),
                        p_dist.sample(&mut rand::rng()),
                    ),
                })
                .insert(Atom)
                .insert(Force::default())
                .insert(Velocity {
                    vel: Vector3::new(
                        v_dist.sample(&mut rand::rng()),
                        v_dist.sample(&mut rand::rng()),
                        v_dist.sample(&mut rand::rng()),
                    )
                    .normalize(),
                })
                .insert(NewlyCreated)
                .insert(Mass { value: 87.0 });
        }

        sim.world_mut().insert_resource(CollisionParameters {
            macroparticle: 5e8,
            box_number: 100, //Any number large enough to cover entire cloud with collision boxes. Overestimating box number will not affect performance.
            box_width: 1e-2, //Too few particles per box will both underestimate collision rate and cause large statistical fluctuations.
            //Boxes must also be smaller than typical length scale of density variations within the cloud, since the collisions model treats gas within a box as homogeneous.
            collision_limit: 10_000_000.0, //Maximum number of collisions that can be calculated in one frame.
                                           //This avoids absurdly high collision numbers if many atoms are initialised with the same position, for example.
        });
        sim.world_mut()
            .insert_resource(CrossSection { sigma: 3.5e-16 });
        sim.add_systems(Update, create_histogram_system);

        sim.insert_resource(Histogram::new(100 as usize));

        sim.world_mut()
            .spawn(WallData {
                wall_type: WallType::Rough,
                wall_temp: Some(TEMP),
                ..Default::default()
            })
            .insert(MySphere { radius: 5e-2 })
            .insert(Position {
                pos: Vector3::new(0.0, 0.0, 0.0),
            });

        sim.world_mut()
            .spawn(Position {
                pos: Vector3::new(0.0, 0.0, 0.0),
            })
            .insert(MySphere { radius: 502e-4 })
            .insert(SimulationVolume {
                volume_type: VolumeType::Inclusive,
            });

        // Define timestep
        sim.world_mut().insert_resource(Timestep { delta: 5e-5 });

        // Run the simulation for a number of steps.
        for _i in 0..5_000 {
            sim.update();
        }
        compare_histogram_to_analytic(&sim.world().get_resource::<Histogram>().unwrap());
    }
}
