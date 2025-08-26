/// Wall collision logic. Does not work with forces/accelertions, or multiple entities.

use std::fmt;
use bevy::prelude::*;
use nalgebra::Vector3;
use rand::Rng;
use rand::distr::Distribution;
use crate::shapes::{
    Cylinder as MyCylinder,
    Cuboid as MyCuboid,
    Sphere as MySphere,
    CylindricalPipe,
    Volume
}; // Aliasing issues with bevy.
use crate::constant::{PI, EXP};
use crate::atom::{Atom, Position, Velocity};
use crate::integrator::{Timestep, AtomECSBatchStrategy,};
use crate::initiate::NewlyCreated;
use super::LambertianProbabilityDistribution;

// let counter: i32 = 0; // Counter for debugging

#[derive(Resource)]
pub struct MaxSteps(pub i32); // Max steps for sdf convergence

#[derive(Resource)]
pub struct SurfaceThreshold(pub f64); // Minimum penetration for detecting a collision

/// Enum for designating wall type for collisions
pub enum WallType {
    // Specular
    Smooth,
    // Diffuse
    Rough,
    // choose randomly between specular and diffuse. Probability of specular reflection.
    Random {spec_prob: f64},
    // choose randomly depending on a coefficient that is exponential in angle of incidence.
    Exponential{value: f64},
}

/// Struct to signify shape is a wall
#[derive(Component)]
#[component(storage = "SparseSet")]
pub struct WallData{
    pub wall_type: WallType,
}

#[derive(Component)]
pub struct DistanceToTravel {
    pub distance_to_travel: f64
}

#[derive(Component, Clone)]
pub struct NumberOfWallCollisions {
    pub value: i32
}

impl fmt::Display for NumberOfWallCollisions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self.value)
    }
}

#[derive(Component)]
pub enum VolumeStatus{
    Inside,
    Outside,
    Open,
}

pub struct CollisionInfo{
    pub atom: Entity,
    pub wall: Entity,
    pub collision_point: Vector3<f64>,
    pub collision_normal: Vector3<f64>
}

pub trait SDF {
    /// Calculate signed distance
    fn signed_distance(&self, point: &Vector3<f64>) -> f64;
}

pub trait Wall {
    /// Collision check for open walls
    fn preliminary_collision_check(&self, 
        atom_pos: &Vector3<f64>, 
        atom_vel: &Vector3<f64>, 
        atom_location: &VolumeStatus,
        wall_pos: &Vector3<f64>, 
        dt: f64) -> bool;
}

pub trait Intersect{
    /// Calculate intersection point between atom trajectory and shape and return point of intersection
    fn calculate_intersect(&self,
        atom_pos: &Vector3<f64>,
        atom_vel: &Vector3<f64>,
        wall_pos: &Vector3<f64>,
        time: f64,
        tolerance: f64,
        max_steps: i32,)
        -> Option<Vector3<f64>>;
}

pub trait Normal{
    /// Calculate normal at point of intersection (collision point)
    fn calculate_normal(&self, point: &Vector3<f64>, wall_pos: &Vector3<f64>, tolerance: f64) -> Option<Vector3<f64>>;
}

/// Credit to Inigo Quilez's for the SDFs. Found at https://www.shadertoy.com/view/Xds3zN

/// Signed distance to a sphere.
impl SDF for MySphere{
    fn signed_distance(&self, point: &Vector3<f64>) -> f64 {
        point.norm() - &self.radius
    }
}

/// Signed distance to an axis-aligned box centered at the origin.
impl SDF for MyCuboid{
    fn signed_distance(&self, point: &Vector3<f64>) -> f64 {
        let delta = Vector3::new(
            point.x.abs() - &self.half_width.x,
            point.y.abs() - &self.half_width.y,
            point.z.abs() - &self.half_width.z,
        );

        let outside = Vector3::new(
            delta.x.max(0.0),
            delta.y.max(0.0),
            delta.z.max(0.0),
        );

        let outside_dist = outside.norm();
        let inside_dist = f64::min(f64::max(delta.x, f64::max(delta.y, delta.z)), 0.0);

        outside_dist + inside_dist
    }
}

/// Signed distance to a capped cylinder.
/// `cyl_start` and `cyl_end` define the axis of the cylinder (centre of the caps).
impl SDF for MyCylinder{
    fn signed_distance(&self, point: &Vector3<f64>) -> f64 {
        let cyl_start = -self.direction*self.length*0.5;
        let local_point = point - cyl_start;

        let point_dot_axis = local_point.dot(&self.direction);

        let radial_dist = (local_point - self.direction * point_dot_axis).norm() - self.radius;

        let axial_dist = (point_dot_axis - self.length * 0.5).abs() - self.length * 0.5;

        let radial_sq = radial_dist * radial_dist;
        let axial_sq = axial_dist * axial_dist;

        let dist = if radial_dist.max(axial_dist) < 0.0 {
            -radial_sq.min(axial_sq)
        } else {
            (if radial_dist > 0.0 { radial_sq } else { 0.0 }) + (if axial_dist > 0.0 { axial_sq } else { 0.0 })
        };

        dist.signum() * dist.abs().sqrt()
    }
}

// Uses -velocity as direction, ignores effect of acceleration.
impl<T : SDF> Intersect for T {
    fn calculate_intersect(
        &self,
        atom_pos: &Vector3<f64>,
        atom_vel: &Vector3<f64>,
        wall_pos: &Vector3<f64>,
        max_time: f64,
        tolerance: f64,
        max_steps: i32,
    ) -> Option<Vector3<f64>> {
        let speed = atom_vel.norm();
        if speed == 0.0 {
            // eprintln!("Speed 0 atom");
            return None;
        }

        let direction = -atom_vel / speed;
        let max_distance = speed * max_time;

        let mut distance_traveled = 0.0;

        for _ in 0..max_steps {
            let curr_pos = atom_pos + direction * distance_traveled;
            let local_pos = curr_pos - wall_pos;

            let distance_to_surface = self.signed_distance(&local_pos);

            if distance_to_surface < tolerance {
                if distance_traveled + distance_to_surface < max_distance + tolerance {
                    return Some(curr_pos);
                }
                else {
                    // eprintln!("Took too long to collide");
                    return None;
                }
            }

            distance_traveled += distance_to_surface;
        }
        // eprintln!("Too few steps for convergence. collision");
        None
    }
}

impl Intersect for CylindricalPipe {
    fn calculate_intersect(
        &self,
        atom_pos: &Vector3<f64>,
        atom_vel: &Vector3<f64>,
        wall_pos: &Vector3<f64>,
        max_time: f64,
        _tolerance: f64,
        _max_steps: i32,
    ) -> Option<Vector3<f64>> {
        let prev_pos = atom_pos - atom_vel * max_time;
        let delta_prev = prev_pos - wall_pos;
        let delta_curr = atom_pos - wall_pos;

        let axial_prev = delta_prev.dot(&self.direction);
        let radial_prev = delta_prev - axial_prev * self.direction;
        let axial_curr = delta_curr.dot(&self.direction);
        let radial_curr = delta_curr - axial_curr * self.direction;

        let a = (radial_curr - radial_prev).norm_squared();
        let b = 2.0 * radial_prev.dot(&(&radial_curr - &radial_prev));
        let c = radial_prev.norm_squared() - self.radius.powi(2);

        // Solve quadratic equation for t
        let discriminant = b * b - 4.0 * a * c;

        if discriminant < 0.0 {
            // No intersection, should never happen
            return None;
        }
        let sqrt_discriminant = discriminant.sqrt();
        let t1 = (-b + sqrt_discriminant) / (2.0 * a);
        let t2 = (-b - sqrt_discriminant) / (2.0 * a);
        if (t1 <= 0.0 || t1 >= 1.0 ) && (t2 <= 0.0 || t2 >= 1.0 ) {
            return None;
        }

        let t = if t1 <= 0.0 || t1 >= 1.0  {
            t2
        } else if t2 <= 0.0 || t2 >= 1.0 {
            t1
        } else {
            f64::min(t1, t2)
        };

        // Calculate the collision point
        let mut collision_point = prev_pos * (1.0 - t) + atom_pos * t;
        collision_point -= (collision_point - wall_pos) * 1e-10; // Jitter to avoid numerical issues
        if (collision_point - wall_pos).dot(&self.direction).abs() <= self.length * 0.5 {
            Some(collision_point)
        } else {
            None
        }
    }
}

impl Normal for MySphere {
    fn calculate_normal(&self, point: &Vector3<f64>, wall_pos: &Vector3<f64>, tolerance: f64) -> Option<Vector3<f64>> {
        let normal_vector = point - wall_pos;

        // Gives the normal assuming collision from the inside
        if (normal_vector.norm() - self.radius).abs() < tolerance  {
            Some(-normal_vector.normalize())
        } else { None } // Should never happen
    }
}

impl Normal for MyCuboid {
    fn calculate_normal(&self, point: &Vector3<f64>, wall_pos: &Vector3<f64>, tolerance: f64) -> Option<Vector3<f64>> {
        let local = point - wall_pos;

        // Gives the normal assuming collision from the inside
        if (local.x.abs() - self.half_width.x).abs() < tolerance {
            Some(-Vector3::new(local.x.signum(), 0.0, 0.0))
        }
        else if (local.y.abs() - self.half_width.y).abs() < tolerance{
            Some(-Vector3::new(0.0, local.y.signum(), 0.0))
        }
        else if (local.z.abs() - self.half_width.z).abs() < tolerance{
            Some(-Vector3::new(0.0, 0.0, local.z.signum()))
        }
        else {
            None
        }
    }
}

impl Normal for MyCylinder {
    fn calculate_normal(&self, point: &Vector3<f64>, wall_pos: &Vector3<f64>, tolerance: f64) -> Option<Vector3<f64>> {
        // Convert to local space
        let rel = point - wall_pos;
        let local = Vector3::new(
            rel.dot(&self.perp_x),
            rel.dot(&self.perp_y),
            rel.dot(&self.direction),
        );

        // Gives the normal assuming collision from the inside
        if (local.z.abs() - self.length*0.5).abs() < tolerance {
            let normal = self.direction * local.z.signum();
            Some(-normal) 
        } else if ((local.x.powf(2.0) + local.y.powf(2.0)).sqrt() - self.radius) < tolerance {
            let normal = self.perp_x * local.x + self.perp_y * local.y;
            Some(-normal.normalize()) 
        }
        else{
            None
        }
    }
}

impl Normal for CylindricalPipe {
    fn calculate_normal(&self, point: &Vector3<f64>, wall_pos: &Vector3<f64>, tolerance: f64) -> Option<Vector3<f64>> {
        // Convert to local space
        let rel = point - wall_pos;
        let local = Vector3::new(
            rel.dot(&self.perp_x),
            rel.dot(&self.perp_y),
            rel.dot(&self.direction),
        );

        if ((local.x.powf(2.0) + local.y.powf(2.0)).sqrt() - self.radius) < tolerance {
            let normal = self.perp_x * local.x + self.perp_y * local.y;
            Some(-normal.normalize()) 
        }
        else{
            None
        }
    }
}

impl<T : Volume> Wall for T {
    fn preliminary_collision_check(&self, 
        atom_pos: &Vector3<f64>, 
        _atom_vel: &Vector3<f64>, 
        atom_location: &VolumeStatus,
        wall_pos: &Vector3<f64>, 
        _dt: f64) -> bool {
        match atom_location {
            VolumeStatus::Inside => {
                return !self.contains(wall_pos, atom_pos);
            }
            VolumeStatus::Outside => {
                return self.contains(wall_pos, atom_pos);
            }
            VolumeStatus::Open => {
                return false;
            }
        }
    }
}

impl Wall for CylindricalPipe {
    fn preliminary_collision_check(&self, 
        atom_pos: &Vector3<f64>, 
        atom_vel: &Vector3<f64>, 
        _atom_location: &VolumeStatus,
        wall_pos: &Vector3<f64>, 
        dt: f64) -> bool {
        let delta_prev = (atom_pos - atom_vel * dt) - wall_pos;
        let axial_prev = delta_prev.dot(&self.direction);
        let radial_prev = delta_prev - axial_prev * self.direction;

        let delta_curr = atom_pos - wall_pos;
        let axial_curr = delta_curr.dot(&self.direction);
        let radial_curr = delta_curr - axial_curr * self.direction;

        let is_within_radius_prev = radial_prev.norm_squared() <= self.radius.powi(2);
        // let is_within_length_prev = f64::abs(axial_prev) <= self.length / 2.0;
        let is_within_radius_curr = radial_curr.norm_squared() <= self.radius.powi(2);
        // let is_within_length_curr = f64::abs(axial_curr) <= self.length / 2.0;

        if is_within_radius_prev {
            return !is_within_radius_curr;
        } else {
            return is_within_radius_curr;
        }
    }
}

/// Specular reflection
fn specular(
    collision_normal: &Vector3<f64>,
    velocity: &Vector3<f64>,
) -> Vector3<f64> {
    let reflected = velocity - 2.0 * velocity.dot(collision_normal) * collision_normal;
    reflected
}

// To be run once at startup
pub fn create_cosine_distribution(mut commands: Commands) {
    // tuple list of (theta, weight)
    let mut thetas = Vec::<f64>::new();
    let mut weights = Vec::<f64>::new();

    // precalculate the discretized cosine distribution.
    let n = 1000; // resolution over which to discretize `theta`.
    for i in 0..n {
        let theta = (i as f64) / (n as f64 + 1.0) * PI / 2.0;
        let weight = theta.cos() * theta.sin();
        thetas.push(theta);
        weights.push(weight);
        // Note: we can exclude d_theta because it is constant and the distribution will be normalized.
    }
    let cosine_distribution = LambertianProbabilityDistribution::new(thetas, weights);
    commands.insert_resource::<LambertianProbabilityDistribution>(cosine_distribution);
    println!("Cosine distribution created!")
}

/// Diffuse collision (Lambertian)
fn diffuse( 
    collision_normal: &Vector3<f64>, 
    velocity: &Vector3<f64>,
    distribution: &LambertianProbabilityDistribution) -> Vector3<f64> {
    // Get random weighted angles
    let phi = rand::rng().random_range(0.0..2.0 * PI);
    let theta = distribution.sample(&mut rand::rng());

    // Get basis vectors
    let dir = Vector3::new(65.514, 5.54, 41.5).normalize();
    let perp_x = collision_normal.cross(&dir).normalize();
    let perp_y = collision_normal.cross(&perp_x).normalize();
    
    // Get scattered velocity
    let x = phi.cos() * theta.sin();
    let y = phi.sin() * theta.sin();
    let z = theta.cos();

    (x * perp_x + y * perp_y + z * collision_normal) * velocity.norm()
}

/// Checks which atoms have collided.
fn collision_check<T: Wall + Intersect + Normal>(
    atom: (Entity, &Position, &Velocity, &VolumeStatus),
    wall: (Entity, &T, &WallData, &Position),       
    dt: f64,
    tolerance: f64, 
    max_steps: i32,
) -> Option<CollisionInfo> {
    let (atom_entity, atom_pos, atom_vel, atom_location) = atom;
    let (wall_entity, shape, wall_data, wall_pos) = wall;

    if !shape.preliminary_collision_check(&atom_pos.pos, &atom_vel.vel, atom_location, &wall_pos.pos, dt) {
        return None;
    }

    // Do collision check
    if let Some(collision_point) = shape.calculate_intersect(&atom_pos.pos, &atom_vel.vel, &wall_pos.pos, dt, tolerance, max_steps) {
        if let Some(collision_normal) = shape.calculate_normal(&collision_point, &wall_pos.pos, tolerance) {
            return Some(CollisionInfo {
                atom: atom_entity,
                wall: wall_entity,
                collision_point,
                collision_normal,
            });
        } else {
            eprintln!(
                "Warning: Could not calculate normal. Atom: {:?}, Wall Pos: {:?}",
                atom_pos.pos, wall_pos.pos
            );
            None
        }
    } else {
        None
    }
}

/// Do wall collision
fn do_wall_collision(
    atom: (&mut Position, &mut Velocity, &mut DistanceToTravel, &mut NumberOfWallCollisions),
    wall: &WallData,
    collision: &CollisionInfo,
    distribution: &LambertianProbabilityDistribution,
    dt: f64,
    tolerance: f64,
) -> f64 {
    let (atom_pos, atom_vel, distance, num_of_collisions) = atom;
    num_of_collisions.value += 1;
    // subtract distance traveled to the collision point from prev position
    let traveled = ((atom_pos.pos - atom_vel.vel*dt) - collision.collision_point).norm();
    distance.distance_to_travel -= traveled;
    if - distance.distance_to_travel > tolerance {
        distance.distance_to_travel = 0.0;
        eprintln!("Distance to travel set to a negative value somehow");
    }
    // println!("original pos {}", (atom_pos.pos - atom_vel.vel*dt));

    // do collision
    match wall.wall_type {
        WallType::Smooth =>
            {atom_vel.vel = specular(&collision.collision_normal, &atom_vel.vel)}
        WallType::Rough =>
            {atom_vel.vel = diffuse(&collision.collision_normal, &atom_vel.vel, &distribution)}
        WallType::Random{spec_prob} => {
            if rand::rng().random_bool(spec_prob) {
                atom_vel.vel = specular(&collision.collision_normal, &atom_vel.vel)
            } else {
                atom_vel.vel = diffuse(&collision.collision_normal, &atom_vel.vel, &distribution)
            }
        }
        WallType::Exponential{value} => {
            let angle_of_incidence = (atom_vel.vel.dot(&collision.collision_normal)/atom_vel.vel.norm()).acos();
            if rand::rng().random_bool((EXP.powf(value * angle_of_incidence) - 1.0)/(EXP.powf(value * PI/2.0) - 1.0) ) {
                atom_vel.vel = specular(&collision.collision_normal, &atom_vel.vel)
            } else {
                atom_vel.vel = diffuse(&collision.collision_normal, &atom_vel.vel, &distribution)
            }
        } 
    }

    if distance.distance_to_travel > 0.0 {
        // propagate along chosen direction
        atom_pos.pos = collision.collision_point + atom_vel.vel.normalize() * distance.distance_to_travel;
    } else {
        atom_pos.pos = collision.collision_point;
    }
    // Return time used
    traveled / atom_vel.vel.norm()
}

/// Initializes distance to travel component
pub fn init_distance_to_travel_system (
    mut query: Query<(Entity, &Velocity), (With<Atom>, Without<DistanceToTravel>)>,
    mut commands: Commands,
    timestep: Res<Timestep>,
) {
    for (atom_entity, vel) in query.iter_mut() {
        commands.entity(atom_entity).insert(DistanceToTravel {
            distance_to_travel: vel.vel.norm() * timestep.delta
        });
    }
}

/// Initializes number of wall collisions component
pub fn init_number_of_collisions_system (
    mut query: Query<(Entity, &Velocity), (With<Atom>, Without<NumberOfWallCollisions>)>,
    mut commands: Commands,
    timestep: Res<Timestep>,
) {
    for (atom_entity, vel) in query.iter_mut() {
        commands.entity(atom_entity).insert(NumberOfWallCollisions {
            value: 0
        });
    }
}

/// Do the collisions
/// To be run after verlet integrate position
pub fn wall_collision_system<T: Wall + Component + Intersect + Normal>(
    wall_query: Query<(Entity, &T, &WallData, &Position), Without<Atom>>,
    mut atom_query: Query<(Entity, 
        &mut Position, 
        &mut Velocity, 
        &mut DistanceToTravel, 
        &mut NumberOfWallCollisions, 
        &VolumeStatus), 
        With<Atom>>,
    batch_strategy: Res<AtomECSBatchStrategy>,
    timestep: Res<Timestep>,
    threshold: Res<SurfaceThreshold>, 
    max_steps: Res<MaxSteps>,
    distribution: Res<LambertianProbabilityDistribution>,
) {
    let tolerance = threshold.0;
    let max_steps = max_steps.0;
    let dt = timestep.delta;
    let walls: Vec<_> = wall_query.iter().collect();
    atom_query
        .par_iter_mut()
        .batching_strategy(batch_strategy.0.clone())
        .for_each(|(atom_entity, mut pos, mut vel, mut distance, mut collisions, location)| {

            let mut dt_atom = dt;
            let mut num_of_collisions = 0;

            while distance.distance_to_travel > 0.0 && num_of_collisions < 10000 { 
                let mut collided = false;
                for (wall_entity, shape, wall, wall_pos) in &walls {
                    if let Some(collision_info) = collision_check(
                        (atom_entity, &pos, &vel, &location),
                        (*wall_entity, *shape, *wall, *wall_pos),
                        dt_atom,
                        tolerance,
                        max_steps,
                    ) {
                        // Handle the collision
                        let time_used = do_wall_collision((&mut pos, &mut vel, &mut distance, &mut collisions), 
                            *wall,
                            &collision_info, 
                            &distribution, 
                            dt_atom, 
                            tolerance);
                        collided = true;
                        dt_atom -= time_used;
                        if dt_atom < 0.0 {
                            dt_atom = 0.0;
                            // eprintln!("Time set to a negative value somehow, wall collision system");
                        }
                    }
                }
                
                if !collided {
                    break;
                }
                num_of_collisions += 1;
            }
        });
}

/// Calculate distance to travel at the end of every frame
/// To be run before wall collision system
pub fn reset_distance_to_travel_system(
    mut query: Query<(&Velocity, &mut DistanceToTravel), With<Atom>>,
    timestep: Res<Timestep>,
) {
    let dt = timestep.delta;

    for (vel, mut distance) in query.iter_mut() {
        distance.distance_to_travel = vel.vel.norm() * dt;
    }
}

/// Assign location status to atoms
pub fn assign_location_status_system(
    mut query: Query<(Entity, &Position), (With<Atom>, With<NewlyCreated>)>,
    spheres: Query<(&MySphere, &Position), With<WallData>>,
    cylinders: Query<(&MyCylinder, &Position), With<WallData>>,
    cuboids: Query<(&MyCuboid, &Position), With<WallData>>,
    mut commands: Commands,
) {
    for (atom_entity, pos) in query.iter_mut() {
        let has_walls = spheres.iter().count() > 0 || 
                        cylinders.iter().count() > 0 || 
                        cuboids.iter().count() > 0;

        let inside = spheres.iter().any(|(wall, wall_pos)| wall.contains(&wall_pos.pos, &pos.pos)) ||
                     cylinders.iter().any(|(wall, wall_pos)| wall.contains(&wall_pos.pos, &pos.pos)) ||
                     cuboids.iter().any(|(wall, wall_pos)| wall.contains(&wall_pos.pos, &pos.pos));

        let status = if has_walls {
            if inside {
                VolumeStatus::Inside
            } else {
                VolumeStatus::Outside
            }
        } else {
            VolumeStatus::Open
        };

        commands.entity(atom_entity).insert(status);
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use rand::Rng;
    use assert_approx_eq::assert_approx_eq;
    use crate::atom::{Atom, Position, Velocity};
    use bevy::prelude::*;
    use crate::shapes::{
        Cylinder as MyCylinder,
        Cuboid as MyCuboid,
        Sphere as MySphere,
        Volume
    };


    const TEST_RUNS: usize = 10_000;
    const TOLERANCE: f64 = 1e-9;
    const MAX_DT: f64 = 1e3;
    const MAX_STEPS: i32 = 1000;

    fn rand_vec(min: f64, max: f64) -> Vector3<f64> {
        let mut rng = rand::rng();
        Vector3::new(
            rng.random_range(min..=max),
            rng.random_range(min..=max),
            rng.random_range(min..=max),
        )
    }

    #[test]
    fn test_intersect_cuboid() {
        let mut success = 0;
        let mut failed = 0;
        for _ in 0..TEST_RUNS {
            let mut rng = rand::rng();
            let length = 1.0;
            let breadth = 1.5;
            let height = 2.0;
            let half_width = Vector3::new(length, breadth, height);

            let cuboid = MyCuboid { half_width };
            let wall_pos = rand_vec(-5.0, 5.0);

            let inside_local = Vector3::new(
                rng.random_range(-half_width.x ..=half_width.x),
                rng.random_range(-half_width.y ..=half_width.y),
                rng.random_range(-half_width.z..=half_width.z),
            );

            let velocity = rand_vec(-1.0, 1.0).normalize();
            let outside_local = inside_local + velocity*10.0;

            let inside = wall_pos + inside_local;
            let outside = wall_pos + outside_local;

            if cuboid.contains(&wall_pos, &outside) || !cuboid.contains(&wall_pos, &inside) {
                panic!("Wrong initialization in test");
            }

            let result = cuboid.calculate_intersect(&outside, &velocity, &wall_pos, MAX_DT, TOLERANCE, MAX_STEPS);

            match result {
                Some(hit) => {
                    let sdf_val = cuboid.signed_distance(&(hit - wall_pos));
                    if sdf_val.abs() < TOLERANCE {
                        success += 1;
                    } else {
                        panic!("[Cuboid] Intersection inaccurate: SDF = {}", sdf_val)
                    }
                }
                None => {
                    failed += 1;
                }
            }
        }

        let failure_rate = (failed as f64 / TEST_RUNS as f64) * 100.0;

        println!(
            "[Cuboid Test] Success: {}, Failed: {}, Failure Rate: {:.2}%",
            success,
            failed,
            failure_rate,
        );

        if failure_rate > 1.0 {
            panic!("Failure rate too high! More than 1%")
        }
    }

    #[test]
    fn test_intersect_sphere() {
        let mut success = 0;
        let mut failed = 0;
        for _ in 0..TEST_RUNS {
            let mut rng = rand::rng();
            let radius = rng.random_range(0.0..10.0);
            let sphere = MySphere { radius };
            let wall_pos = rand_vec(-5.0, 5.0);

            let radius_in = rng.random_range(0.0..radius);
            let inside_local = rand_vec(-1.0,1.0).normalize() * radius_in;

            let velocity = rand_vec(-1.0, 1.0).normalize();
            let outside_local = inside_local + velocity*radius*2.0;

            let inside = wall_pos + inside_local;
            let outside = wall_pos + outside_local;

            if sphere.contains(&wall_pos, &outside) || !sphere.contains(&wall_pos, &inside) {
                panic!("Wrong initialization in test");
            }

            let result = sphere.calculate_intersect(&outside, &velocity, &wall_pos, MAX_DT, TOLERANCE, MAX_STEPS);

            match result {
                Some(hit) => {
                    let sdf_val = sphere.signed_distance(&(hit - wall_pos));
                    if sdf_val.abs() < TOLERANCE {
                        success += 1;
                    } else {
                        panic!("[Sphere] Intersection inaccurate: SDF = {}", sdf_val)
                    }
                }
                None => {
                    failed += 1;
                }
            }
        }

        let failure_rate = (failed as f64 / TEST_RUNS as f64) * 100.0;

        println!(
            "[Sphere Test] Success: {}, Failed: {}, Failure Rate: {:.2}%",
            success,
            failed,
            failure_rate,
        );

        if failure_rate > 1.0 {
            panic!("Failure rate too high! More than 1%")
        }

    }

    #[test]
    fn test_intersect_cylinder() {
        let mut success = 0;
        let mut failed = 0;
        for _ in 0..TEST_RUNS {
            let mut rng = rand::rng();
            let radius = rng.random_range(0.1..10.0);
            let length = rng.random_range(1.0..10.0);

            // Random direction vector
            let random_dir;
            random_dir = rand_vec(-1.0, 1.0);

            let wall_pos = rand_vec(-5.0, 5.0);
            let cylinder = MyCylinder::new(radius, length, random_dir);

            let inside_local = {
                let radial_in = rng.random_range(0.0..radius);
                let angle_in = rng.random_range(0.0..std::f64::consts::TAU);
                let x_in = radial_in * angle_in.cos();
                let y_in = radial_in * angle_in.sin();
                let z_in = rng.random_range(-length * 0.4..length * 0.4);
                let inside = cylinder.perp_x * x_in + cylinder.perp_y * y_in + cylinder.direction * z_in;
                inside
            };

            let velocity = rand_vec(-1.0, 1.0).normalize();
            let outside_local = inside_local + velocity*22.5;

            let inside = wall_pos + inside_local;
            let outside = wall_pos + outside_local;

            if cylinder.contains(&wall_pos, &outside) || !cylinder.contains(&wall_pos, &inside) {
                panic!("Wrong initialization in test");
            }

            let result = cylinder.calculate_intersect(&outside, &velocity, &wall_pos, MAX_DT, TOLERANCE, MAX_STEPS);

            match result {
                Some(hit) => {
                    let sdf_val = cylinder.signed_distance(&(hit - wall_pos));
                    if sdf_val.abs() < TOLERANCE {
                        success += 1;
                    } else {
                        panic!("[Cylinder] Intersection inaccurate: SDF = {}", sdf_val)
                    }
                }
                None => {
                    failed += 1;
                }
            }
        }

        let failure_rate = (failed as f64 / TEST_RUNS as f64) * 100.0;

        println!(
            "[Cylinder Test] Success: {}, Failed: {}, Failure Rate: {:.2}%",
            success,
            failed,
            failure_rate,
        );

        if failure_rate > 1.0 {
            panic!("Failure rate too high! More than 1%")
        }
    }

    #[test]
    fn test_normal_sphere() {
        for _ in 0..TEST_RUNS {
            let mut rng = rand::rng();
            let radius = rng.random_range(0.1..10.0);
            let sphere = MySphere { radius };
            let wall_pos = rand_vec(-5.0, 5.0);
            
            let inside_local = rand_vec(-1.0, 1.0).normalize() * radius;
            let inside = wall_pos + inside_local;

            let result = sphere.calculate_normal(&inside, &wall_pos, TOLERANCE);
            
            match result {
                Some(normal) => {
                    assert_approx_eq!(-inside_local.normalize()[0], normal[0], 1e-12);
                    assert_approx_eq!(-inside_local.normalize()[1], normal[1], 1e-12);
                    assert_approx_eq!(-inside_local.normalize()[2], normal[2], 1e-12);
                }
                None => panic!("Normal calculation failed for sphere"),
            }
        }
    }

    #[test]
    fn test_normal_cuboid() {
        for _ in 0..TEST_RUNS {
            let mut rng = rand::rng();
            let length = rng.random_range(0.0..10.0);
            let breadth = rng.random_range(0.0..10.0);
            let height = rng.random_range(0.0..10.0);
            let half_width = Vector3::new(length, breadth, height);

            let cuboid = MyCuboid { half_width };
            let wall_pos = rand_vec(-5.0, 5.0);
        
            let face = rng.random_range(0..6); 
        
            let (inside_local, expected_normal) = match face {
                0 => (Vector3::new(length, rng.random_range(-breadth.. breadth), rng.random_range(-height..height)), Vector3::new(-1.0,0.0,0.0)),// Positive X face
                1 => (Vector3::new(-length, rng.random_range(-breadth.. breadth), rng.random_range(-height..height)),Vector3::new(1.0,0.0,0.0)), // Negative X face
                2 => (Vector3::new(rng.random_range(-length..length), breadth, rng.random_range(-height..height)), Vector3::new(0.0,-1.0,0.0)),// Positive Y face
                3 => (Vector3::new(rng.random_range(-length..length), -breadth, rng.random_range(-height..height)),Vector3::new(0.0,1.0,0.0)), // Negative Y face
                4 => (Vector3::new(rng.random_range(-length..length), rng.random_range(-breadth.. breadth), height), Vector3::new(0.0,0.0,-1.0)),// Positive Z face
                5 => (Vector3::new(rng.random_range(-length..length), rng.random_range(-breadth.. breadth), -height),Vector3::new(0.0,0.0,1.0)), // Negative Z face
                _ => panic!("Invalid face selection"), // This should never happen
            };
        
            let inside = wall_pos + inside_local;
        
            let result = cuboid.calculate_normal(&inside, &wall_pos, TOLERANCE);
        
            match result {
                Some(normal) => {
                    assert_approx_eq!(normal[0], expected_normal[0], 1e-12);
                    assert_approx_eq!(normal[1], expected_normal[1], 1e-12);
                    assert_approx_eq!(normal[2], expected_normal[2], 1e-12);
                }
                None => panic!("Normal calculation failed for cuboid"),
            }
        }
    }
    
    // This is more complicated than the thing its testing
    #[test]
    fn test_normal_cylinder() {
        for _ in 0..TEST_RUNS {
            let mut rng = rand::rng();
            let radius = 1.0;
            let length = 2.0;
            let random_dir = Vector3::new(0.0,0.0,1.0); // Random direction
            let cylinder = MyCylinder::new(radius, length, random_dir);
            let wall_pos = rand_vec(-5.0, 5.0);
    
            let face = rng.random_range(0..3);
    
            let (surface_local, expected_normal) = match face {
                0 => {  // Curved face
                    let angle = rng.random_range(0.0..std::f64::consts::TAU);
                    let x = radius * angle.cos();
                    let y = radius * angle.sin();
                    let z = rng.random_range(-length*0.5..length*0.5);
                    let surface_local = x * cylinder.perp_x + y * cylinder.perp_y + z * cylinder.direction;
                    let normal = -(x * cylinder.perp_x + y * cylinder.perp_y).normalize();
                    (surface_local, normal) 
                },
                1 => {  // Top cap
                    let radial = rng.random_range(0.0..radius);
                    let angle = rng.random_range(0.0..std::f64::consts::TAU);
                    let x = radial * angle.cos();
                    let y = radial * angle.sin();
                    let z = length * 0.5;
                    let surface_local = x*cylinder.perp_x + y*cylinder.perp_y + z*cylinder.direction;
                    (surface_local, -random_dir)
                },
                2 => {  // Bottom cap 
                    let radial = rng.random_range(0.0..radius);
                    let angle = rng.random_range(0.0..std::f64::consts::TAU);
                    let x = radial * angle.cos();
                    let y = radial * angle.sin();
                    let z = -length * 0.5; 
                    let surface_local = x*cylinder.perp_x + y*cylinder.perp_y + z*cylinder.direction;
                    (surface_local, random_dir) 
                },
                _ => panic!("Invalid face selection"),
            };
            
            if cylinder.signed_distance(&surface_local).abs() > TOLERANCE {
                println!("{}", cylinder.signed_distance(&surface_local));
                panic!("Surface point not initialized correctly fix test")
            }

            let surface = wall_pos + surface_local;
    
            let result = cylinder.calculate_normal(&surface, &wall_pos, TOLERANCE);
    
            match result {
                Some(normal) => {
                    assert_approx_eq!(normal[0], expected_normal[0], 1e-12);
                    assert_approx_eq!(normal[1], expected_normal[1], 1e-12);
                    assert_approx_eq!(normal[2], expected_normal[2], 1e-12);
                }
                None => panic!("Normal calculation failed for cylinder"),
            }
        }
    }

    // this is ugly as sin
    #[test]
    fn test_collision_check() {
        let mut app = App::new();
        let position1 = Position {pos: Vector3::new(5.0, 0.0, 0.0)};
        let velocity1= Velocity {vel: Vector3::new(1.0, 0.0, 0.0)};
        let dt = 1.0 + 1e-15;
        let atom1 = app.world_mut().spawn(position1.clone()).insert(velocity1.clone()).insert(Atom).insert(VolumeStatus::Inside).id();

        let wall_pos1 = Position {pos: Vector3::new(0.0, 0.0, 0.0)};
        let sphere = MySphere{radius: 4.0 };
        let wall_type = WallData {
            wall_type: WallType::Smooth};
        let sphere_wall = app.world_mut()
            .spawn(wall_pos1.clone())
            .insert(WallData {
                wall_type: WallType::Smooth})
            .insert(MySphere{radius: 4.0 })
            .id();

        let result1 = collision_check((atom1, &position1, &velocity1, &VolumeStatus::Inside), 
            (sphere_wall, &sphere, &wall_type, &wall_pos1), 
            dt, TOLERANCE, MAX_STEPS);
        match result1 {
            Some(collision_info) => {
                assert_eq!(atom1, collision_info.atom);
                assert_eq!(sphere_wall, collision_info.wall);
                assert_approx_eq!(4.0, collision_info.collision_point[0], TOLERANCE);
                assert_approx_eq!(0.0, collision_info.collision_point[1], TOLERANCE);
                assert_approx_eq!(0.0, collision_info.collision_point[2], TOLERANCE);
                assert_approx_eq!(-1.0, collision_info.collision_normal[0], 1e-14);
                assert_approx_eq!(0.0, collision_info.collision_normal[1], 1e-14);
                assert_approx_eq!(0.0, collision_info.collision_normal[2], 1e-14);
            }
            None => {
                panic!();
            }
        }

        app.world_mut().despawn(atom1);
        app.world_mut().despawn(sphere_wall);


        let position2 = Position {pos: Vector3::new(5.0, 0.0, 0.0)};
        let velocity2 = Velocity {vel: Vector3::new(1.0, 0.0, 0.0)};
        let dt = 1.0;
        let atom2 = app.world_mut().spawn(position2.clone()).insert(velocity2.clone()).insert(Atom).insert(VolumeStatus::Inside).id();

        let wall_pos2 = Position {pos: Vector3::new(0.0, 0.0, 0.0)};
        let cuboid = MyCuboid{half_width: Vector3::new(4.0, 1.0, 1.0)};
        let wall_type = WallData {
            wall_type: WallType::Smooth};
        let cuboid_wall = app.world_mut()
            .spawn(wall_pos2.clone())
            .insert(WallData {

                wall_type: WallType::Smooth})
            .insert(MyCuboid{half_width: Vector3::new(4.0, 1.0, 1.0)})
            .id();

        let result2 = collision_check((atom2, &position2, &velocity2, &VolumeStatus::Inside), 
            (cuboid_wall, &cuboid, &wall_type, &wall_pos2), 
            dt, TOLERANCE, MAX_STEPS);
        match result2 {
            Some(collision_info) => {
                assert_eq!(atom2, collision_info.atom);
                assert_eq!(cuboid_wall, collision_info.wall);
                assert_approx_eq!(4.0, collision_info.collision_point[0], TOLERANCE);
                assert_approx_eq!(0.0, collision_info.collision_point[1], TOLERANCE);
                assert_approx_eq!(0.0, collision_info.collision_point[2], TOLERANCE);
                assert_approx_eq!(-1.0, collision_info.collision_normal[0], 1e-14);
                assert_approx_eq!(0.0, collision_info.collision_normal[1], 1e-14);
                assert_approx_eq!(0.0, collision_info.collision_normal[2], 1e-14);
            }
            None => {
                panic!();
            }
        }

        app.world_mut().despawn(atom2);
        app.world_mut().despawn(cuboid_wall);

        let position3 = Position {pos: Vector3::new(5.0, 0.0, 0.0)};
        let velocity3 = Velocity {vel: Vector3::new(1.0, 0.0, 0.0)};
        let dt = 1.0 + 1e-15;
        let atom3 = app.world_mut().spawn(position3.clone()).insert(velocity3.clone()).insert(Atom).insert(VolumeStatus::Inside).id();

        let wall_pos3 = Position {pos: Vector3::new(0.0, 0.0, 0.0)};
        let cylinder = MyCylinder::new(1.0, 8.0, Vector3::new(1.0, 0.0, 0.0));
        let wall_type = WallData {
            wall_type: WallType::Smooth};
        let cylinder_wall = app.world_mut()
            .spawn(wall_pos3.clone())
            .insert(WallData {
                wall_type: WallType::Smooth})
            .insert(MyCylinder::new(1.0, 8.0, Vector3::new(1.0, 0.0, 0.0)))
            .id();

        let result3 = collision_check((atom3, &position3, &velocity3, &VolumeStatus::Inside), 
            (cylinder_wall, &cylinder, &wall_type, &wall_pos3), 
            dt, TOLERANCE, MAX_STEPS);
        match result3 {
            Some(collision_info) => {
                assert_eq!(atom3, collision_info.atom);
                assert_eq!(cylinder_wall, collision_info.wall);
                assert_approx_eq!(4.0, collision_info.collision_point[0], TOLERANCE);
                assert_approx_eq!(0.0, collision_info.collision_point[1], TOLERANCE);
                assert_approx_eq!(0.0, collision_info.collision_point[2], TOLERANCE);
                assert_approx_eq!(-1.0, collision_info.collision_normal[0], 1e-14);
                assert_approx_eq!(0.0, collision_info.collision_normal[1], 1e-14);
                assert_approx_eq!(0.0, collision_info.collision_normal[2], 1e-14);
            }
            None => {
                panic!();
            }
        }

        app.world_mut().despawn(atom3);
        app.world_mut().despawn(cylinder_wall);

        let position4 = Position {pos: Vector3::new(2.5, 1.0, 1.0)};
        let velocity4 = Velocity {vel: Vector3::new(1.0, 0.0, 0.0)};
        let dt = 1.0 + 1e-15;
        let atom4 = app.world_mut().spawn(position4.clone()).insert(velocity4.clone()).insert(Atom).id();

        let wall_pos4 = Position {pos: Vector3::new(1.0, 1.0, 1.0)};
        let nozzle = CylindricalPipe::new(1.0, 8.0, Vector3::new(0.0, 1.0, 0.0));
        let wall_type = WallData {
            wall_type: WallType::Smooth};
        let pipe_wall = app.world_mut()
            .spawn(wall_pos4.clone())
            .insert(WallData {
                wall_type: WallType::Smooth})
            .insert(CylindricalPipe::new(1.0, 8.0, Vector3::new(0.0, 1.0, 0.0)))
            .id();

        let result4 = collision_check((atom4, &position4, &velocity4, &VolumeStatus::Inside), 
            (pipe_wall, &nozzle, &wall_type, &wall_pos4), 
            dt, TOLERANCE, MAX_STEPS);
        match result4 {
            Some(collision_info) => {
                assert_eq!(atom4, collision_info.atom);
                assert_eq!(pipe_wall, collision_info.wall);
                assert_approx_eq!(2.0, collision_info.collision_point[0], TOLERANCE);
                assert_approx_eq!(1.0, collision_info.collision_point[1], TOLERANCE);
                assert_approx_eq!(1.0, collision_info.collision_point[2], TOLERANCE);
                assert_approx_eq!(-1.0, collision_info.collision_normal[0], 1e-14);
                assert_approx_eq!(0.0, collision_info.collision_normal[1], 1e-14);
                assert_approx_eq!(0.0, collision_info.collision_normal[2], 1e-14);
            }
            None => {
                panic!();
            }
        }
    }

    // so is this
    #[test]
    fn test_do_wall_collision() {
        let mut app = App::new();
        let dt = 1.0 - 1e-10;
        let position = Position {pos: Vector3::new(1.0, 1.0, 0.0)};
        let velocity = Velocity {vel: Vector3::new(2.0,2.0,0.0)};
        let length = velocity.vel.norm()*dt;
        let distance = DistanceToTravel{distance_to_travel: length};
        let atom = app.world_mut()
            .spawn(Atom)
            .insert(position.clone())
            .insert(velocity.clone())
            .insert(DistanceToTravel{distance_to_travel: length})
            .insert(NumberOfWallCollisions{value: 0})
            .insert(VolumeStatus::Inside)
            .id();
        let wall = app.world_mut().spawn(WallData {
            wall_type: WallType::Smooth}).id();
        let collision_point = Vector3::new(0.0,0.0,0.0);
        let collision_normal = Vector3::new(-1.0,0.0,0.0);

        fn test_system(
            mut query: Query<(Entity, &mut Position, &mut Velocity, &mut DistanceToTravel, &mut NumberOfWallCollisions)>,
            wall: Query<(Entity, &WallData)>,
            distribution: Res<LambertianProbabilityDistribution>,
            tolerance: Res<SurfaceThreshold>) {
            query.iter_mut().for_each(|(atom, mut atom_pos, mut atom_vel, mut distance, mut collisions)| {
                for (wall_entity, wall) in wall.iter() {
                    let dt = 1.0 - 1e-10; // If you change this make sure to change the dt above as well
                    let collision_point = Vector3::new(0.0,0.0,0.0);
                    let collision_normal = Vector3::new(-1.0,0.0,0.0);
                    let collision_info = CollisionInfo {
                        atom,
                        wall: wall_entity,
                        collision_point,
                        collision_normal,
                    };
                    let time_used = do_wall_collision((&mut atom_pos, &mut atom_vel, &mut distance, &mut collisions), 
                        wall,  
                        &collision_info, 
                        &distribution, 
                        dt, 
                        tolerance.0);
                    assert_approx_eq!(time_used, ((atom_pos.pos - atom_vel.vel*dt) - collision_point).norm()/atom_vel.vel.norm())
                }
            });
        }
        app.add_systems(Startup, create_cosine_distribution);
        app.add_systems(Update, test_system);
        app.world_mut().insert_resource(SurfaceThreshold(1e-9));
        app.update();

        let new_velocity = app.world()
            .entity(atom)
            .get::<Velocity>()
            .expect("Entity not found!")
            .vel;
        let new_position = app.world()
            .entity(atom)
            .get::<Position>()
            .expect("Entity not found")
            .pos;
        let new_distance = app.world()
            .entity(atom)
            .get::<DistanceToTravel>()
            .expect("Entity not found")
            .distance_to_travel;
        let new_number_of_wall_collisions = app.world()
            .entity(atom)
            .get::<NumberOfWallCollisions>()
            .expect("Entity not found")
            .value;
            
        let time_used = ((position.pos - velocity.vel*dt) - collision_point).norm()/velocity.vel.norm();
        let distance_travelled = ((position.pos - velocity.vel*dt) - collision_point).norm();

        assert_ne!(new_velocity, velocity.vel);
        assert_eq!(new_number_of_wall_collisions, 1);
        assert_approx_eq!(new_position[0], collision_point[0] + new_velocity[0]*(dt - time_used), 1e-15);
        assert_approx_eq!(new_position[1], collision_point[1] + new_velocity[1]*(dt - time_used), 1e-15);
        assert_approx_eq!(new_position[2], collision_point[2] + new_velocity[2]*(dt - time_used), 1e-15);
        assert_approx_eq!(distance.distance_to_travel - distance_travelled, new_distance, 1e-15);
    }

    // Test multiple collisions
    // Basically an integration test
    #[test]
    fn test_systems() {
        use crate::integrator::{IntegrationPlugin, IntegrationSet, Timestep, AtomECSBatchStrategy, OldForce};
        use crate::initiate::NewlyCreated;
        use crate::atom::{Force, Mass};
        use crate::collisions::ApplyAtomCollisions;
        use crate::collisions::CollisionPlugin;

        let mut app = App::new();
        app.world_mut()
            .spawn(WallData{wall_type: WallType::Smooth})
            .insert(CylindricalPipe::new(1.0, 8.0, Vector3::new(0.0, 0.0, 1.0)))
            .insert(Position{pos: Vector3::new(0.0, 0.0, 0.0)});
        let atom = app.world_mut()
            .spawn(Atom)
            .insert(Position{pos: Vector3::new(-1.0 + 1e-8, 0.0, 0.0)})
            .insert(Velocity{vel: Vector3::new(0.0, -2.5 * PI, 0.0)})
            .insert(NewlyCreated)
            .insert(Force::default())
            .insert(Mass { value: 1.0 })
            .insert(OldForce(Force::default()))
            .id();
        app.add_plugins(IntegrationPlugin);
        app.add_plugins(CollisionPlugin);
        app.world_mut().insert_resource(ApplyAtomCollisions(false));
        app.world_mut().insert_resource(Timestep {delta: 1.0});
        app.update();

        let new_position = app.world()
            .entity(atom)
            .get::<Position>()
            .expect("Entity not found")
            .pos;

        let wall_collisions = app.world()
            .entity(atom)
            .get::<NumberOfWallCollisions>()
            .expect("Entity not found")
            .value;

        println!("{}", new_position);
        println!("wall collisions {}", wall_collisions); 
        assert_approx_eq!(new_position[0], 0.0, 1e-5);
        assert_approx_eq!(new_position[1], -1.0, 1e-5);
        assert_approx_eq!(new_position[2], 0.0, 1e-5);
    }
}


