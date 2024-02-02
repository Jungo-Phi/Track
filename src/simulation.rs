use std::time::Duration;
use eframe::egui::{Color32, Painter, Rounding};
use nalgebra::Vector2;
use ndarray::{Array1, Array2};
use crate::camera::Camera;
use crate::constrains::Constrain;
use crate::force_generators::ForceGenerator;
use crate::particle::Particle;
use crate::rect::MyRect;
use crate::utils::{lineal_solver, THIN_BLACK_STROKE};


/// **My Float** is the type used for all calculations in the physics engine
pub type MyF = f64;


#[derive(Clone)]
pub struct SimulationCondition {
	name: String,
	particles: Vec<Particle>,
	constrains: Vec<Box<dyn Constrain + Send>>,
	force_generators: Vec<Box<dyn ForceGenerator + Send>>,
	bounding_box: Option<MyRect>,
}

impl SimulationCondition {
	pub fn new(name: String,
	           particles: Vec<Particle>,
	           constrains: Vec<Box<dyn Constrain + Send>>,
	           force_generators: Vec<Box<dyn ForceGenerator + Send>>,
	           bounding_box: Option<MyRect>) -> Self {
		Self { name, particles, constrains, force_generators, bounding_box }
	}
	
	pub fn get_name(&self) -> &String { &self.name }
}

/// Simulation contains all the variables related to the simulation.
///
/// It's basically the physics engine.
///
/// It is made to work with rigid bodies.
/// - Collisions
/// - Constrains (ex: pivots)
pub struct Simulation {
	time: Duration,
	particles: Vec<Particle>,
	lambda: Array1<f64>,
	conditions: SimulationCondition,
}

impl Simulation {
	const KS: f64 = 2.;
	const KD: f64 = 2.;
	
	pub fn new(mut conditions: SimulationCondition) -> Self {
		// Initialize the constrains
		conditions.constrains.iter_mut()
		          .for_each(|constrain| constrain.init(&conditions.particles));
		
		Self {
			time: Duration::ZERO,
			particles: conditions.particles.iter().map(|particle| particle.clone()).collect(),
			lambda: default_lambda(conditions.constrains.len()),
			conditions,
		}
	}
	
	pub fn get_time(&self) -> &Duration {
		&self.time
	}
	pub fn reset(&mut self) {
		self.particles = self.conditions.particles.clone();
		self.lambda = default_lambda(self.conditions.constrains.len());
		self.time = Duration::ZERO;
	}
	pub fn change_conditions(&mut self, conditions: SimulationCondition) {
		self.conditions = conditions;
		self.reset();
	}
	
	pub fn update(&mut self, delta: Duration) {
		self.time += delta;
		let delta = delta.as_secs_f64();
		
		// 1 - Clear forces
		self.particles.iter_mut().for_each(|particle| {
			particle.clear_force_accumulator();
		});
		// 2 - Apply forces from force generators
		// self.mouse_spring.apply_forces(&mut self.particles);
		self.conditions.force_generators.iter_mut().for_each(|force_generator| {
			force_generator.apply_forces(&mut self.particles);
		});
		// 3 - Apply constraint forces (or reaction forces)
		let constrain_forces = self.get_constrain_forces();
		for (particle, force) in self.particles.iter_mut().zip(constrain_forces.into_iter()) {
			particle.apply_force(force);
		}
		// 4 - Update the particles
		self.particles.iter_mut().for_each(|particle| {
			particle.update(delta, self.conditions.bounding_box);
		});
	}
	
	pub fn draw(&self, painter: &mut Painter, camera: &Camera, show_forces: bool, show_speeds: bool) {
		if let Some(bounding_box) = self.get_bounding_box() {
			// camera.fill_rect(painter, bounding_box, Rounding::none(), Color32::WHITE);
			camera.stroke_rect(painter, bounding_box, Rounding::none(), THIN_BLACK_STROKE);
		}
		
		self.particles.iter()
		    .for_each(|particle| particle.draw(painter, camera));
		self.conditions.constrains.iter()
		    .for_each(|constrain| constrain.draw(painter, camera, &self.particles));
		self.conditions.force_generators.iter()
		    .for_each(|force_generator| force_generator.draw(painter, camera, &self.particles));
		if show_forces {
			self.particles.iter()
			    .for_each(|particle| particle.draw_forces(painter, camera, 0.02, Color32::GREEN));
		}
		if show_speeds {
			self.particles.iter()
			    .for_each(|particle| particle.draw_velocity(painter, camera, 0.2, Color32::LIGHT_RED));
		}
	}
	
	pub fn kinetic_energy(&self) -> MyF {
		self.particles.iter()
		    .map(|particle| particle.kinetic_energy()).sum()
	}
	pub fn potential_energy(&self) -> MyF {
		self.conditions.force_generators.iter()
		    .map(|force_generator| force_generator.potential_energy(&self.particles)).sum()
	}
	pub fn get_bounding_box(&self) -> Option<MyRect> { self.conditions.bounding_box }
	
	fn get_constrain_forces(&mut self) -> Vec<Vector2<f64>> {
		let particle_size = 2 * self.particles.len();
		let constrain_size = self.conditions.constrains.len();
		
		// State vector v of the velocity of the particles [v1x, v1y, v2x, v2y, ...]
		let mut v_vector = Array1::<f64>::zeros(particle_size);
		for (index, particle) in self.particles.iter().enumerate() {
			v_vector[2 * index] = particle.get_velocity().x as f64;
			v_vector[2 * index + 1] = particle.get_velocity().y as f64;
		}
		// Matrix w (w = 1/m * I)   the identity matrix times one over the masses
		// [[1 / m1, 0     , 0     , ...],
		//  [0     , 1 / m1, 0     , ...],
		//  [0     , 0     , 1 / m2, ...],
		//  [...   , ...   , ...   , ...]]
		let mut w_matrix = Array2::<f64>::zeros((particle_size, particle_size));
		for (index, particle) in self.particles.iter().enumerate() {
			let w = 1. / particle.get_mass() as f64;
			w_matrix[[2 * index, 2 * index]] = w;
			w_matrix[[2 * index + 1, 2 * index + 1]] = w;
		}
		// Vector f (Q) of all the forces
		let mut f_vector = Array1::<f64>::zeros(particle_size);
		for (index, particle) in self.particles.iter().enumerate() {
			f_vector[2 * index] = particle.get_force().x as f64;
			f_vector[2 * index + 1] = particle.get_force().y as f64;
		}
		// Vector c of constrains {constrain_size}
		let mut c_vector = Array1::<f64>::zeros(constrain_size);
		for (index, constrain) in self.conditions.constrains.iter().enumerate() {
			c_vector[index] = constrain.constrain_function(&self.particles) as f64;
		}
		
		// Vector c_derivative of constrains {constrain_size}
		let mut c_derivative = Array1::<f64>::zeros(constrain_size);
		for (index, constrain) in self.conditions.constrains.iter().enumerate() {
			c_derivative[index] = constrain.constrain_derivative(&self.particles) as f64;
		}
		// Matrix J is the jacobian of the constrains (dc/dq)
		// [[dc1/dx1, dc1/dy1, dc1/dx2, dc1/dy2, ...],
		//  [dc2/dx1, dc2/dy1, dc2/dx2, dc2/dy2, ...],
		//  [...    ,...     ,...     ,...     , ...]]
		let mut j_matrix = Array2::<f64>::zeros((constrain_size, particle_size));
		for (constrain_index, constrain) in self.conditions.constrains.iter().enumerate() {
			let j = constrain.jacobian_blocs(&self.particles);
			for (particle_index, jacobian_bloc) in j.iter() {
				j_matrix[[constrain_index, *particle_index]] = *jacobian_bloc as f64;
			}
		}
		// Derivative of the jacobian
		let mut j_derivative = Array2::<f64>::zeros((constrain_size, particle_size));
		for (constrain_index, constrain) in self.conditions.constrains.iter().enumerate() {
			let j = constrain.jacobian_derivative_blocs(&self.particles);
			for (particle_index, jacobian_bloc) in j.iter() {
				j_derivative[[constrain_index, *particle_index]] = *jacobian_bloc as f64;
			}
		}
		// Left side (A) of the equation J * W * Jt
		let a = j_matrix.dot(&w_matrix).dot(&j_matrix.t());
		// Right side (B) of the equation -J. * v - J * W * f - Ks * c - Kd * c_derivative
		let b = -j_derivative.dot(&v_vector)
			- j_matrix.dot(&w_matrix).dot(&f_vector)
			- Self::KS * &c_vector
			- Self::KD * &c_derivative;
		
		let lambda = lineal_solver::gauss_seidel(a, b, self.lambda.clone(), 0.1, 20);
		
		let reaction_vector = j_matrix.t().dot(&lambda);
		self.lambda = lambda;
		
		// let lambda = Array1::<f64>::zeros(particle_size);
		let mut reaction_forces = Vec::<Vector2<f64>>::new();
		for (index, _particle) in self.particles.iter().enumerate() {
			let reaction_force = Vector2::new(reaction_vector[2 * index] as f64, reaction_vector[2 * index + 1] as f64);
			reaction_forces.push(reaction_force);
		}
		
		// let j = Vec2::new(*j_matrix.get((0, 2)).unwrap() as f32, *j_matrix.get((0, 3)).unwrap() as f32);
		// let force = self.particles[1].get_force();
		// println!("force = {:?}", force);
		// let reaction_force_2 = -force.projected_onto(j);
		// println!("reaction force 1 = {:?}", reaction_forces[1]);
		// println!("reaction force 2 = {:?}", reaction_force_2);
		// reaction_forces[1] = reaction_force_2;
		reaction_forces
	}
}


fn default_lambda(n: usize) -> Array1<MyF> {
	Array1::<MyF>::from_elem(n, 1.)
}
