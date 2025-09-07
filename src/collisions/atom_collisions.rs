//! Implements s-wave scattering of atoms
//! We use here a standard Direct Simulation Monte Carlo method of simulating collisions. For much greater detail on these alogrithms see e.g.
//! Molecular Gas Dynamics and the Direct Simulation of Gas Flows 1998 by G.A. Bird.
//! We here divide the space into a grid of collision cells within which collisiosn can occur. Based on simple kinetic theory we predict how many collisions
//! should occur within each box based on density and average velocity, and randomly select this many pairs of atoms to collide.
//!
//! # Limitations
//! We assume the atoms within a cell have an approximately thermal distribution in order to relate average velocity to average relative velocity.
//! For cases where this approximation is poor, the collision rate may be wrong.
//! We assume a single species of atom, with a constant (not velocity dependent) collisional cross-section.
//! We cannot have too few particles per box: the current implementation allows the same number of particles to collide multiple times, 
//! the effect of which is exacerbated in cells with few particles.
//!
//!

use hashbrown::HashMap;
use nalgebra::Vector3;
use rand::Rng;
use bevy::prelude::*;
use crate::atom::{Position, Velocity, Atom};
use crate::constant::{PI, SQRT2};
use crate::integrator::Timestep;
use crate::collisions::spatial_grid::BoxID;

/// A patition of space within which collisions can occur
pub struct CollisionBox {
    pub entity_velocities: Vec<(Entity, Vector3<f64>)>,
    pub expected_collision_checks: f64,
    pub collision_number: i32,
    pub density: f64,
    pub volume: f64,
    pub atom_number: f64,
    pub particle_number: i32,
}

impl Default for CollisionBox {
    fn default() -> Self {
        CollisionBox {
            entity_velocities: Vec::new(),
            expected_collision_checks: 0.0,
            density: 0.0,
            volume: 0.0,
            atom_number: 0.0,
            collision_number: 0,
            particle_number: 0,
        }
    }
}

impl CollisionBox {
    /// Perform collisions within a box.
    fn do_collisions(&mut self, params: CollisionParameters, dt: f64) {
        let mut rng = rand::rng();
        self.particle_number = self.entity_velocities.len() as i32;
        self.atom_number = self.particle_number as f64 * params.macroparticle;

        // Only one atom or less in box - no collisions.
        if self.particle_number <= 1 {
            return;
        }

        // find the max speed in the box to use the No Time Counter method 
        let mut max_speed = self.entity_velocities
            .iter()
            .map(|(_, velocity)| velocity.norm())
            .fold(0.0, f64::max);
    

        // number of times we check for collisions is N*n*sigma*v*dt, where n is atom density and N is atom number
        // probability of any given pair of particles colliding is sigma*vrel/(sigma*vrel)_max, sigma cross section and vrel the relative speed
        // and since we assume these are identical particles we must divide by two since otherwise we count each collision twice
        let density = self.atom_number / params.box_width.powi(3);
        self.expected_collision_checks =
            (self.particle_number as f64 - 1.0) * density * params.sigma * max_speed * dt;

        let mut num_checks_left: f64 = self.expected_collision_checks;

        if num_checks_left > params.collision_limit {
            panic!("Number of collisions in a box in a single frame exceeds limit. Number of collisions={}, limit={}, particles={}.", num_checks_left, params.collision_limit, self.particle_number);
        }

        while num_checks_left > 0.0 {
            let check = if num_checks_left > 1.0 {
                true
            } else {
                rng.random_bool(num_checks_left)
            };

            if check {
                let idx1 = rng.random_range(0..self.entity_velocities.len());
                let mut idx2 = rng.random_range(0..self.entity_velocities.len() - 1);
                if idx2 >= idx1 {
                    idx2 = idx2 + 1;
                }

                let v1 = self.entity_velocities[idx1].1;
                let v2 = self.entity_velocities[idx2].1;
                let vel_rel = (v1 - v2).norm();
                if vel_rel > 2.0 * max_speed {
                    println!("!");
                    num_checks_left += (self.particle_number as f64 - 1.0) * density * params.sigma * (vel_rel - 2.0 * max_speed) * dt / 2.0 ;
                    max_speed = vel_rel / 2.0;
                }
                let prob_collision =  vel_rel / (2.0 * max_speed);
                // println!("{}", (v1 -v2).norm());
                let collide = rng.random_bool(prob_collision);
                if collide {
                    let (v1new, v2new) = do_collision(v1, v2);
                    self.entity_velocities[idx1].1 = v1new;
                    self.entity_velocities[idx2].1 = v2new;
                    self.collision_number += 1;
                }
            }

            num_checks_left -= 1.0;
        }
        /////
    }
}

/// Resource for defining collision relevant paramaters like macroparticle number, box width and number of boxes
#[derive(Copy, Clone, Resource)]
pub struct CollisionParameters {
    /// number of real particles one simulation particle represents for collisions
    pub macroparticle: f64,
    /// number of boxes per side in spatial binning
    pub box_number: i64,
    /// width of one box in m
    pub box_width: f64,
    // collisional cross section of atoms (assuming only one species)
    pub sigma: f64,
    /// Limit on number of collisions per box each frame. If the number of collisions to calculate exceeds this, the simulation will panic.
    pub collision_limit: f64,
}

/// Resource that stores stats about collisions
#[derive(Clone, Resource)]
pub struct CollisionsTracker {
    /// number of collisions in each box
    pub num_collisions: Vec<i32>,
    /// number of simulated particles in each box
    pub num_particles: Vec<i32>,
    /// number of simulated atoms in each box
    pub num_atoms: Vec<f64>,
}

/// Performs collisions within the atom cloud using a spatially partitioned Monte-Carlo approach.
pub fn apply_collisions_system(
    mut query: Query<(Entity, &Position, &mut Velocity, &mut BoxID), With<Atom>>,
    timestep: Res<Timestep>,
    params: Res<CollisionParameters>,
    mut tracker: ResMut<CollisionsTracker>,
) {
    use rayon::prelude::*;
    // insert atom velocity into hash
    let mut map: HashMap<i64, CollisionBox> = HashMap::new();
    for (entity, _, velocity, boxid) in query.iter() {
        if boxid.id == i64::MAX {
            continue;
        } else {
            map.entry(boxid.id).or_default()
                .entity_velocities.push((entity, velocity.vel));
        }
    }

    // get immutable list of boxes and iterate in parallel
    // (Note that using hashmap parallel values mut does not work in parallel, tested.)
    let boxes: Vec<&mut CollisionBox> = map.values_mut().collect();
    boxes.into_par_iter().for_each(|collision_box| {
        collision_box.do_collisions(*params, timestep.delta);
    });

    // Write back values.
    for collision_box in map.values() {
        for &(entity, new_velocity) in &collision_box.entity_velocities {
            if let Ok((_, _, mut velocity, _)) = query.get_mut(entity) {
                velocity.vel = new_velocity;
            }
        }
    }

    // Update tracker
    tracker.num_atoms = map
        .values()
        .map(|collision_box| collision_box.atom_number)
        .collect();
    tracker.num_collisions = map
        .values()
        .map(|collision_box| collision_box.collision_number)
        .collect();
    tracker.num_particles = map
        .values()
        .map(|collision_box| collision_box.particle_number)
        .collect();
}


fn do_collision(mut v1: Vector3<f64>, mut v2: Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    let mut rng = rand::rng();

    // Randomly modify velocities in CoM frame, conserving energy & momentum
    let vcm = 0.5 * (v1 + v2);
    let energy: f64 = 0.5 * ((v1 - vcm).norm().powi(2) + (v2 - vcm).norm().powi(2));

    let cos_theta: f64 = rng.random_range(-1.0..1.0);
    let sin_theta: f64 = (1.0 - cos_theta.powi(2)).sqrt();
    let phi: f64 = rng.random_range(0.0..2.0 * PI);

    let v_prime = Vector3::new(
        energy.sqrt() * sin_theta * phi.cos(),
        energy.sqrt() * sin_theta * phi.sin(),
        energy.sqrt() * cos_theta,
    );
    v1 = vcm + v_prime;
    v2 = vcm - v_prime;

    (v1, v2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::atom::{Atom, Force, Mass, Position, Velocity};
    use crate::initiate::NewlyCreated;
    use crate::integrator::Timestep;
    use crate::collisions::CollisionPlugin;
    use nalgebra::Vector3;
    use crate::simulation::SimulationBuilder;

    #[test]
    fn test_do_collision() {
        // do this test muliple times since there is a random element involved in do_collision
        let v1 = Vector3::new(0.5, 1.0, 0.75);
        let v2 = Vector3::new(0.2, 0.0, 1.25);
        //calculate energy and momentum before
        let ptoti = v1 + v2;
        let energyi = 0.5 * (v1.norm_squared() + v2.norm_squared());
        for _i in 0..50 {
            let (v1new, v2new) = do_collision(v1, v2);

            //energy and momentum after
            let ptotf = v1new + v2new;
            let energyf = 0.5 * (v1new.norm_squared() + v2new.norm_squared());

            assert!((ptoti - ptotf) <= Vector3::new(1e-6, 1e-6, 1e-6));
            assert!((energyi - energyf) / energyi <= 1e-12);
            assert_ne!(v1, v1new);
            assert_ne!(v2, v2new);
        }
    }

    /// Test that the expected number of collisions in a CollisionBox is correct.
    /// This will fail sometimes
    #[test]
    fn test_collision_rate() {
        use assert_approx_eq::assert_approx_eq;
        let mut rng = rand::rng();
        let params = CollisionParameters {
            macroparticle: 2.0,
            box_number: 1,
            box_width: 1e-2,
            sigma: 5e-10,
            collision_limit: 1_000_000.0,
        };
        const MACRO_ATOM_NUMBER: usize = 100_000;
        let dt = 1e-3;
        let mut expected_collision = 0.0;
        let mut actual_collision = 0.0;
        for _i in 0..100 {
            let x1 = rng.random::<f64>();
            let x2 = rng.random::<f64>();
            let v1 = Vector3::new(x1 * 2.0, 0.0, 0.0);
            let v2 = Vector3::new(x2 * 2.0, 0.0, 0.0);
            
            // Create dummy entities for testing
            let mut entity_velocities: Vec<(Entity, Vector3<f64>)> = Vec::new();
            for _i in 0..MACRO_ATOM_NUMBER {
                let entity = Entity::PLACEHOLDER;
                entity_velocities.push((entity, v1));
                let entity = Entity::PLACEHOLDER;
                entity_velocities.push((entity, v2));
            }
            
            let mut collision_box = CollisionBox {
                entity_velocities,
                ..Default::default()
            };
            
            collision_box.do_collisions(params, dt);

            let atom_number = params.macroparticle * 2.0 * MACRO_ATOM_NUMBER as f64;
            let density = atom_number / params.box_width.powi(3);
            let expected_number = (collision_box.particle_number as f64 - 1.0) * density * params.sigma * (v1 - v2).norm() * dt / 4.0;
            expected_collision += expected_number;
            actual_collision += collision_box.collision_number as f64;
            assert_eq!(collision_box.particle_number, 2 * MACRO_ATOM_NUMBER as i32);
            assert_eq!(collision_box.atom_number, atom_number);
            assert_approx_eq!(collision_box.collision_number as f64, expected_number, expected_number * 0.15);
        }
        let delta = (expected_collision - actual_collision).abs()/expected_collision;
        if delta > 0.05 {
            panic!()
        }
        println!("{}", delta);
    }

    /// Test that the system runs and causes nearby atoms to collide. More of an integration test than a unit test.
    #[test]
    fn test_collisions() {
        let mut simulation_builder = SimulationBuilder::default();
        // simulation_builder.add_end_frame_systems();
        simulation_builder.add_plugins(CollisionPlugin);
        let mut sim = simulation_builder.build();

        let vel1 = Vector3::new(1.0, 0.0, 0.0);
        let vel2 = Vector3::new(-1.0, 0.0, 0.0);

        let pos1 = Vector3::new(-3.0, 0.0, 0.0);
        let pos2 = Vector3::new(3.0, 0.0, 0.0);

        //atom 1 to collide
        let atom1 = sim
            .world_mut()
            .spawn(Velocity { vel: vel1 })
            .insert(Position { pos: pos1 })
            .insert(Atom)
            .insert(Force::default())
            .insert(Mass { value: 87.0 })
            .insert(NewlyCreated)
            .id();

        //atom2 to collide
        let atom2 = sim
            .world_mut()
            .spawn(Velocity { vel: vel2 })
            .insert(Position { pos: pos2 })
            .insert(Atom)
            .insert(Force::default())
            .insert(Mass { value: 87.0 })
            .insert(NewlyCreated)
            .id();

        let dt = 1.0;
        sim.world_mut().insert_resource(Timestep { delta: dt });
        sim.world_mut().insert_resource(CollisionsTracker {
            num_collisions: Vec::new(),
            num_atoms: Vec::new(),
            num_particles: Vec::new(),
        });
        sim.world_mut().insert_resource(CollisionParameters {
            macroparticle: 1.0,
            box_number: 10,
            box_width: 2.0,
            sigma: 10.0,
            collision_limit: 10_000.0,
        });

        for _i in 0..10 {
            sim.update();
        }

        let vel1new = sim.world().entity(atom1).get::<Velocity>().expect("atom1 not found");
        let vel2new = sim.world().entity(atom2).get::<Velocity>().expect("atom2 not found");

        let pos1new = sim.world().entity(atom1).get::<Position>().expect("atom1 not found");
        let pos2new = sim.world().entity(atom2).get::<Position>().expect("atom2 not found");

        assert_ne!(pos1, pos1new.pos);
        assert_ne!(pos2, pos2new.pos);

        assert_ne!(vel1 - vel1new.vel, Vector3::new(0.0, 0.0, 0.0));
        assert_ne!(vel2 - vel2new.vel, Vector3::new(0.0, 0.0, 0.0));
    }
}
