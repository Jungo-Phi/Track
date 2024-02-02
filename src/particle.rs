#![allow(unused)]
use eframe::egui::{Color32, Painter, Stroke};
use nalgebra::{Point2, Vector2};
use crate::camera::Camera;
use crate::rect::MyRect;
use crate::simulation::MyF;
use crate::utils::{GRAVITY, integrator, REBOUND_DAMPING, THIN_BLACK_STROKE, UP};

#[derive(Debug, Clone)]
/// A Particle is an object that have a mass, a position, a velocity, and respond to forces.
///
/// It has no shape, but it can be drawn as a circle.
///
/// It is the basic element of a physics engine.
pub struct Particle {
	mass: MyF,
	position: Point2<MyF>,
	velocity: Vector2<MyF>,
	force_accumulator: Vec<Vector2<MyF>>,
	color: Color32,
}

impl Particle {
	pub const RADIUS: MyF = 0.10;
	pub const GRAB_RADIUS: f32 = 100.;
	
	pub fn new(mass: MyF, position: Point2<MyF>, velocity: Vector2<MyF>, color: Color32) -> Self {
		Self {
			mass,
			position,
			velocity,
			force_accumulator: Vec::new(),
			color,
		}
	}
	
	pub fn get_position(&self) -> Point2<MyF> {
		self.position
	}
	//pub fn set_position(&mut self, position: Point2<MyF>) { self.position = position; }
	pub fn get_velocity(&self) -> Vector2<MyF> {
		self.velocity
	}
	//pub fn set_velocity(&mut self, velocity: Vector2<MyF>) { self.velocity = velocity; }
	pub fn get_mass(&self) -> MyF {
		self.mass
	}
	
	pub fn apply_force(&mut self, force: Vector2<MyF>) {
		self.force_accumulator.push(force);
	}
	pub fn clear_force_accumulator(&mut self) { self.force_accumulator.clear() }
	
	pub fn kinetic_energy(&self) -> MyF {
		0.5 * self.get_mass() * self.get_velocity().magnitude_squared()
	}
	
	pub fn get_force(&self) -> Vector2<MyF> { self.force_accumulator.clone().into_iter().sum::<Vector2<MyF>>() }
	// pub fn collide_point(&self, point: Point2<MyF>) -> bool { (self.position - point).norm() < Self::RADIUS }
	
	fn solve_collision_rect(&mut self, rect: MyRect, delta: MyF, delta_position: Vector2<MyF>, delta_velocity: Vector2<MyF>) {
		self.position += delta_position;
		self.velocity += delta_velocity;
		
		// bounce against a vertical wall
		if self.position.x < rect.left() {
			self.velocity.x *= -REBOUND_DAMPING;
			self.position.x = (1. + REBOUND_DAMPING) * rect.left() - REBOUND_DAMPING * self.position.x;
		} else if self.position.x > rect.right() {
			self.velocity.x *= -REBOUND_DAMPING;
			self.position.x = (1. + REBOUND_DAMPING) * rect.right() - REBOUND_DAMPING * self.position.x;
		}
		
		// bounce against a horizontal wall
		if self.position.y < rect.bottom() {
			// bounce against the floor
			
			// self.velocity.y *= -REBOUND_DAMPING;
			self.position.y = (1. + REBOUND_DAMPING) * rect.bottom() - REBOUND_DAMPING * self.position.y;
			
			// let force = UP * (self.mass * (1. + REBOUND_DAMPING) * -self.velocity.y / delta);
			// self.velocity += delta * force / self.mass;
			self.velocity += UP * ((1. + REBOUND_DAMPING) * -self.velocity.y);
		} else if self.position.y > rect.top() {
			self.velocity.y *= -REBOUND_DAMPING;
			self.position.y = (1. + REBOUND_DAMPING) * rect.top() - REBOUND_DAMPING * self.position.y;
		}
	}
	
	pub fn update(&mut self, delta: MyF, bounding_box: Option<MyRect>) {
		let acceleration = self.get_force() / self.mass;
		let (delta_position, delta_velocity) = integrator::rk2(self.velocity, acceleration, delta);
		if let Some(bounding_box) = bounding_box {
			self.solve_collision_rect(bounding_box.cropped(Self::RADIUS), delta, delta_position, delta_velocity);
		} else {
			self.position += delta_position;
			self.velocity += delta_velocity;
		}
	}
	
	pub fn draw(&self, painter: &mut Painter, camera: &Camera) {
		camera.fill_circle(painter, self.position, Self::RADIUS, self.color);
		camera.stroke_circle(painter, self.position, Self::RADIUS, THIN_BLACK_STROKE);
		/*
		let vertices = Vec::from([
			Point2::new(0.2, 1.1), Point2::new(0.7, 0.7), Point2::new(0.3, 0.2), Point2::new(0.1, 0.5)
		]);
		self.camera.paint_polygon(painter, vertices.clone(), Color32::BLUE.linear_multiply(0.4), THIN_BLACK_STROKE);
		vertices.iter().for_each(|&vertex| self.camera.paint_point(painter, vertex, 5., Color32::BLUE));
		 */
	}
	
	pub fn draw_forces(&self, painter: &mut Painter, camera: &Camera, scale: MyF, color: Color32) {
		let start = self.position;
		self.force_accumulator.iter().for_each(|&force| {
			let end = start + force * scale;
			camera.paint_wire_arrow(painter, start, end, Stroke::new(2., color));
		});
	}
	pub fn draw_velocity(&self, painter: &mut Painter, camera: &Camera, scale: MyF, color: Color32) {
		let start = self.position;
		let end = start + self.velocity * scale;
		camera.paint_wire_arrow(painter, start, end, Stroke::new(2., color));
	}
}
