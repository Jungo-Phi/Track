#![allow(dead_code, unused)]

use std::f64::consts::TAU;
use std::ops::RangeInclusive;
use dyn_clone::{clone_trait_object, DynClone};
use eframe::egui::{Color32, lerp, Painter, Stroke};
use nalgebra::{Affine2, Isometry2, Matrix3, Point2, Similarity2, Vector2};
use crate::camera::Camera;
use crate::particle::Particle;
use crate::simulation::MyF;
use crate::utils::{angle_of, THIN_BLACK_STROKE, VEC_X};

pub trait ForceGenerator: DynClone {
	fn apply_forces(&self, particles: &mut Vec<Particle>);
	fn potential_energy(&self, particles: &Vec<Particle>) -> MyF;
	fn draw(&self, painter: &mut Painter, camera: &Camera, particles: &Vec<Particle>);
}

clone_trait_object!(ForceGenerator);


#[derive(Debug, Clone)]
pub struct Constant {
	acceleration: Vector2<f64>,
}

impl Constant {
	pub fn new(acceleration: Vector2<f64>) -> Self { Self { acceleration } }
}

impl ForceGenerator for Constant {
	fn apply_forces(&self, particles: &mut Vec<Particle>) {
		particles.iter_mut().for_each(|particle| {
			particle.apply_force(particle.get_mass() * self.acceleration);
		});
	}
	
	fn potential_energy(&self, particles: &Vec<Particle>) -> MyF {
		particles.iter().map(|particle| -particle.get_mass() * self.acceleration.dot(&particle.get_position().coords)).sum()
	}
	
	fn draw(&self, painter: &mut Painter, camera: &Camera, particles: &Vec<Particle>) {}
}

/// A physics_engine is a force generator.
///
/// It attracts or repels 2 particles (end1 and end2), forcing them to maintain a fixed distance.
///
/// The quantities of the physics_engine are:
/// - the **rest length**
/// - the force constant **k** (N/m)
/// - the damping **b** (kg/s)
#[derive(Debug, Clone)]
pub struct Spring {
	end1_index: usize,
	end2_index: usize,
	/// Spring stiffness
	k: f64,
	/// Spring damping
	b: f64,
	/// No force is applied when the two particles are at this distance.
	rest_length: f64,
}

impl Spring {
	pub fn new(end1_index: usize, end2_index: usize, k: f64, b: f64, default_length: f64) -> Self {
		Self { rest_length: default_length, end1_index, end2_index, k, b }
	}
	pub fn set_end2_index(&mut self, end2_index: usize) {
		self.end2_index = end2_index;
	}
	pub fn get_end2_index(&mut self) -> usize {
		self.end2_index // TODO <- find a better way to grab particles
	}
}

impl Spring {
	const RADIUS: MyF = 0.08;
}

impl ForceGenerator for Spring {
	fn apply_forces(&self, particles: &mut Vec<Particle>) {
		if self.end1_index == self.end2_index {
			return;
		}
		
		let delta_position = particles[self.end2_index].get_position() - particles[self.end1_index].get_position();
		let delta_velocity = particles[self.end2_index].get_velocity() - particles[self.end1_index].get_velocity();
		let direction = delta_position.normalize();
		
		let force = -self.k * (delta_position.norm() - self.rest_length);
		let damping = -self.b * delta_velocity.dot(&direction);
		let force = direction * (force + damping);
		
		particles[self.end1_index].apply_force(-force);
		particles[self.end2_index].apply_force(force);
	}
	
	fn potential_energy(&self, particles: &Vec<Particle>) -> MyF {
		let delta = particles[self.end2_index].get_position() - particles[self.end1_index].get_position();
		0.5 * self.k * (delta.norm() - self.rest_length).powi(2)
	}
	
	fn draw(&self, painter: &mut Painter, camera: &Camera, particles: &Vec<Particle>) {
		let start = particles[self.end1_index].get_position();
		let end = particles[self.end2_index].get_position();
		let delta = end - start;
		
		let n = 1.max((self.rest_length / Self::RADIUS / 3.) as i32);
		
		let scaling = delta.norm() - 2. * Self::RADIUS;
		
		let mut points = Vec::new();
		points.push(Point2::origin());
		(0..n).for_each(|i| {
			points.push(Point2::new((4 * i + 1) as MyF / (4 * n) as MyF, Self::RADIUS / scaling));
			points.push(Point2::new((4 * i + 3) as MyF / (4 * n) as MyF, -Self::RADIUS / scaling));
		});
		points.push(Point2::new(1., 0.));
		
		let transform = Similarity2::new(
			start.coords + Self::RADIUS * delta / delta.norm(), angle_of(delta), scaling
		);
		
		points.iter_mut().for_each(|mut point| *point = transform.transform_point(point));
		
		points.insert(0, start);
		points.push(end);
		
		camera.paint_lines(painter, points, Stroke::new(2., Color32::BLACK));
		
		/*
		let n = 5; // Number of turns
		let d = 0.1; // Small diameter in proportion to large diameter
		let m = 1.4; // Width of the end parts in proportion to small diameter
		let f = 0.5; // Length of the end parts in proportion to large diameter
		
		let non_uniform_scaling = |x_scaling: f64, y_scaling: f64| {
			Affine2::from_matrix_unchecked(Matrix3::new(x_scaling, 0., 0., 0., y_scaling, 0., 0., 0., 1.))
		};

		if delta.norm() / self.diameter <= 2. * f {
			let transform = Isometry2::new(start_position.coords, delta.get_angle())
				* non_uniform_scaling(self.diameter * f, self.diameter);

			let o = delta.norm() / self.diameter - 2. * f + 1.;
			let mut poly = Vec::from([
				Point2::new(o - d * m / f, -d * m),
				Point2::new(o - d * m / f, -0.5),
				Point2::new(o + d * m / f, -0.5),
				Point2::new(o + d * m / f, -d * m),
			]);
			poly.extend(
				(0..=5)
					.map(|i| {
						let v = Vector2::new_unitary((i as f64 * 0.1 - 0.25) * TAU);
						Point2::new(v.x / f, v.y) * d * m + Vector2::new(o * 2., 0.)
					})
					.collect::<Vec<Point2<f64>>>(),
			);
			poly.extend(Vec::from([
				Point2::new(o + d * m / f, d * m),
				Point2::new(o + d * m / f, 0.5),
				Point2::new(o - d * m / f, 0.5),
				Point2::new(o - d * m / f, d * m),
			]));
			poly.extend(
				(0..=5)
					.map(|i| {
						let v = Vector2::new_unitary((i as f64 * 0.1 + 0.25) * TAU);
						Point2::new(v.x / f, v.y) * d * m
					})
					.collect::<Vec<Point2<f64>>>(),
			);

			let poly = poly.iter().map(|point| transform * point).collect();
			camera.fill_polygon(painter, self.color, &poly);
			camera.draw_polygon(painter, Color32::BLACK, &poly);
		} else {
			let spacing = 1. / (2 * n + 1) as f64;
			let dl = (delta.norm() / self.diameter - 2.) * spacing;
			let ld = d * self.diameter * (dl * dl + 1.).sqrt();
			let darker_color = darker(self.color, 0.8);

			let start_transform = Isometry2::new(start_position.coords, delta.get_angle())
				* non_uniform_scaling(self.diameter * f, self.diameter);
			let end_transform = Isometry2::new(end_position.coords, delta.get_angle())
				* non_uniform_scaling(-self.diameter * f, self.diameter);

			let isometry = Isometry2::new(start_position.coords + start_transform * Vector2::x(), delta.get_angle());
			let scaling = non_uniform_scaling(delta.norm() - self.diameter * f * 2., self.diameter * 0.5);

			// Draws the back spires
			(0..=n).for_each(|i| {
				let m = (2 * i) as f64 * spacing;
				let p1 = scaling * Point2::new(m, 1.);
				let p2 = scaling * Point2::new(m + spacing, -1.);

				let poly = Vec::from([
					p1 + Vector2::new(-ld, 0.),
					p1 + Vector2::new(ld, 0.),
					p2 + Vector2::new(ld, 0.),
					p2 + Vector2::new(-ld, 0.),
				])
				.iter()
				.map(|point| isometry * point)
				.collect();

				camera.fill_polygon(painter, darker_color, &poly);
				camera.draw_polygon(painter, Color32::BLACK, &poly);
			});
			// Draws the front spires
			(0..n).for_each(|i| {
				let m = ((2 * i + 1) as f64) * spacing;
				let p1 = scaling * Point2::new(m, -1.);
				let p2 = scaling * Point2::new(m + spacing, 1.);

				let poly = Vec::from([
					p1 + Vector2::new(-ld, 0.),
					p1 + Vector2::new(ld, 0.),
					p2 + Vector2::new(ld, 0.),
					p2 + Vector2::new(-ld, 0.),
				])
				.iter()
				.map(|point| isometry * point)
				.collect();

				camera.fill_polygon(painter, self.color, &poly);
				camera.draw_polygon(painter, Color32::BLACK, &poly);
			});

			// Draws the ends
			let mut poly = Vec::from([
				Point2::new(1. - d * m / f, -d * m),
				Point2::new(1. - d * m / f, -0.5),
				Point2::new(1. + d * m / f, -0.5),
				Point2::new(1. + d * m / f, 0.5),
				Point2::new(1. - d * m / f, 0.5),
				Point2::new(1. - d * m / f, d * m),
			]);
			let poly_r: Vec<Point2<f64>> = (0..=5)
				.map(|i| {
					let v = Vector2::new_unitary((i as f64 * 0.1 + 0.25) * TAU);
					Point2::new(v.x / f, v.y) * d * m
				})
				.collect();
			poly.extend(poly_r);

			let poly_start = poly.iter().map(|point| start_transform * point).collect();
			camera.fill_polygon(painter, self.color, &poly_start);
			camera.draw_polygon(painter, Color32::BLACK, &poly_start);
			let poly_end = poly.iter().map(|point| end_transform * point).collect();
			camera.fill_polygon(painter, self.color, &poly_end);
			camera.draw_polygon(painter, Color32::BLACK, &poly_end);
		}
	 */
	}
}

/// A motor is a force generator.
///
/// It applies a force to a particle (start) to make it rotate around another particle (end).
///
/// It is generally used in conjunction with a `Rod` to make a motorized joint.
#[derive(Debug, Clone)]
pub struct Motor {
	start: usize,
	end: usize,
	speed: f64,
	color: Color32,
}

impl Motor {
	pub fn new(start: usize, end: usize, speed: f64, color: Color32) -> Self {
		Self { start, end, speed, color }
	}
}

impl ForceGenerator for Motor {
	fn apply_forces(&self, particles: &mut Vec<Particle>) {
		let start_position = particles[self.start].get_position();
		let end_position = particles[self.end].get_position();
		let delta_position = end_position - start_position;
		
		// particles[self.end].position = start_position + delta_position.rotated(self.speed * delta);
		//1 particles[self.end].apply_force(delta_position.perpendicular() * self.speed);
	}
	
	fn potential_energy(&self, particles: &Vec<Particle>) -> MyF {
		0.
	}
	
	fn draw(&self, painter: &mut Painter, camera: &Camera, particles: &Vec<Particle>) {
		panic!("not implamented!");
		/*
		let start_position = particles[self.start].get_position();
		let end_position = particles[self.end].get_position();
		let delta_position = end_position - start_position;
		let radius = delta_position.norm();
		let angle = delta_position.get_angle().to_degrees();

		let w = 5;

		if radius as i16 - w <= 0 {
			return;
		}
		/*
				DrawRenderer::filled_circle(
					painter,
					start_position.x as i16,
					start_position.y as i16,
					radius as i16 + w,
					self.color,
				)
				.unwrap();
		*/
		DrawRenderer::circle(
			painter,
			start_position.x as i16,
			start_position.y as i16,
			radius as i16 - w,
			Color32::BLACK,
		)
		.unwrap();

		DrawRenderer::circle(
			painter,
			start_position.x as i16,
			start_position.y as i16,
			radius as i16 + w,
			Color32::BLACK,
		)
		.unwrap();

		/*
		(0..4).into_iter().for_each(|i| {
			let angle = angle + i as f64 / 4. * 360.;
			let t1 = start_position + Vector2::from_polar_deg(radius + w as f64 - 1., angle);
			let t2 = start_position + Vector2::from_polar_deg(radius + w as f64 - 2., angle + 45.);
			DrawRenderer::thick_line(
				painter,
				start_position.x as i16,
				start_position.y as i16,
				t1.x as i16,
				t1.y as i16,
				2 * w as u8,
				Color32::BLACK,
			)
			.unwrap();
			DrawRenderer::filled_pie(
				painter,
				t1.x as i16,
				t1.y as i16,
				w,
				angle as i16 - 90,
				angle as i16 + 90,
				Color32::BLACK,
			)
			.unwrap();
			DrawRenderer::filled_pie(
				painter,
				t2.x as i16,
				t2.y as i16,
				w * 2,
				angle as i16 - 45,
				angle as i16 + 135,
				self.color,
			)
			.unwrap();
			DrawRenderer::pie(
				painter,
				t2.x as i16,
				t2.y as i16,
				w * 2,
				angle as i16 - 45,
				angle as i16 + 135,
				Color32::BLACK,
			)
			.unwrap();
		});
		 */
		*/
	}
}
