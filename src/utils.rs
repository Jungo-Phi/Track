#![allow(unused)]
use eframe::egui::{Color32, Stroke};
use eframe::emath::{Pos2, Vec2};
use nalgebra::{ArrayStorage, Const, Matrix, Matrix2, Point2, Vector2};
use crate::simulation::MyF;


pub const GRAVITY: MyF = 9.81;
pub const REBOUND_DAMPING: MyF = 1.;

pub const THIN_BLACK_STROKE: Stroke = Stroke { width: 1., color: Color32::BLACK };
pub const MIRROR_Y: Matrix<MyF, Const<2>, Const<2>, ArrayStorage<MyF, 2, 2>> = Matrix2::new(1., 0.0, 0., -1.);

pub const VEC_X: Vector2<MyF> = Vector2::new(1., 0.);
pub const VEC_Y: Vector2<MyF> = Vector2::new(0., 1.);
pub const RIGHT: Vector2<MyF> = Vector2::new(1., 0.);
pub const LEFT: Vector2<MyF> = Vector2::new(-1., 0.);
pub const UP: Vector2<MyF> = Vector2::new(0., 1.);
pub const DOWN: Vector2<MyF> = Vector2::new(0., -1.);


pub fn to_pos2(point2: Point2<MyF>) -> Pos2 {
	Pos2 { x: point2.x as f32, y: point2.y as f32 }
}

pub fn to_vec2(vector2: Vector2<MyF>) -> Vec2 {
	Vec2 { x: vector2.x as f32, y: vector2.y as f32 }
}

pub fn to_point2(pos2: Pos2) -> Point2<MyF> {
	Point2::new(pos2.x as MyF, pos2.y as MyF)
}

pub fn to_vector2(vec2: Vec2) -> Vector2<MyF> {
	Vector2::new(vec2.x as MyF, vec2.y as MyF)
}


pub fn scale2unit(log_scale: i16) -> &'static str {
	match log_scale {
		-3 => "km", -2 => "hm", -1 => "dam",
		0 => "m",
		1 => "dm", 2 => "cm", 3 => "mm",
		4 => "μm", 5 => "nm", 6 => "pm",
		_ => "ERROR",
	}
}


/// 90° rotation in the trigonometric direction
pub fn perpendicular(vec: Vector2<MyF>) -> Vector2<MyF> {
	Vector2::new(-vec.y, vec.x)
}

/// The angle of the vector in radians from (x=1, y=0) in the trigonometric direction
pub fn angle_of(vec: Vector2<MyF>) -> MyF {
	MyF::atan2(vec.y, vec.x)
}


/// Return the last n elements of a vector
pub fn vec_last<T: Clone>(vector: Vec<T>, n: usize) -> Vec<T> {
	if n >= vector.len() { return vector; }
	Vec::from(&vector[(vector.len() - n)..])
}


/// **Explicits ODE solvers**
///
/// Those fonctions are explicits methods for numerical integration.
///
/// Ordinary differential equation (ODE) solvers.
///
/// Used to predict the new position and velocity of an object after a time step (delta)
/// ```
/// rk2(position, velocity, acceleration, delta);
/// ```
pub mod integrator {
	use nalgebra::{Point2, Vector2};
	use crate::simulation::MyF;
	
	/// Euler method, simple but with high error
	pub fn euler(position: &mut Point2<MyF>, velocity: &mut Vector2<MyF>, acceleration: Vector2<MyF>, delta: MyF) {
		*position += delta * *velocity;
		*velocity += delta * acceleration;
	}
	
	/// Runge-Kutta 2nd order (midpoint method)
	///
	/// In this case,
	/// witch is the movement of an object in a cartesian coordinate system,
	/// this method is already exact.
	///
	/// So there's no need to use rk4 or any higher order method.
	pub fn rk2(velocity: Vector2<MyF>, acceleration: Vector2<MyF>, delta: MyF) -> (Vector2<MyF>, Vector2<MyF>) {
		(delta * (velocity + 0.5 * delta * acceleration), delta * acceleration)
	}
	
	/// Runge-Kutta 4th order (Recommended)
	/// is far less error prone than euler.
	///
	/// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
	///
	/// Where the parameter 'derivative' is a function of 'value' d/dt(value)
	/// ```
	/// rk1(
	///   position,
	///   velocity,
	///   delta,
	///   |position: MyF, velocity: MyF, acceleration: MyF,
	///    delta: MyF, ki: (MyF, MyF)| {
	/// 	let sx = position + ki.0 * delta;
	/// 	let sv = velocity + ki.1 * delta;
	/// 	(sv, acceleration) // k(i+1)
	///   }
	/// );
	/// ```
	pub fn rk4(position: &mut MyF, velocity: &mut MyF, acceleration: MyF, delta: MyF,
	           integration: Box<dyn Fn(MyF, MyF, MyF, (MyF, MyF)) -> (MyF, MyF)>) {
		let k0: (MyF, MyF) = (0., 0.); // (velocity, acceleration)
		let k1: (MyF, MyF) = integration(*position, *velocity, delta * 0., k0);
		let k2: (MyF, MyF) = integration(*position, *velocity, delta * 0.5, k1);
		let k3: (MyF, MyF) = integration(*position, *velocity, delta * 0.5, k2);
		let k4: (MyF, MyF) = integration(*position, *velocity, delta * 1., k3);
		
		*position += delta * (k1.0 + 2.0 * (k2.0 + k3.0) + k4.0) / 6.0;
		*velocity += delta * (k1.1 + 2.0 * (k2.1 + k3.1) + k4.1) / 6.0;
	}
}


/// **Linear solver**
///
/// Those fonctions are linear solvers for linear systems of equations.
///
/// Used to find the constrains forces
/// ```
/// rk2(position, velocity, acceleration, delta);
/// ```
pub mod lineal_solver {
	use ndarray::{Array1, Array2};
	
	use crate::simulation::MyF;

	/// Gauss-Seidel method for solving linear systems of equations (Ax = b).
	///
	/// https://en.wikipedia.org/wiki/Gauss%E2%80%93Seidel_method
	pub fn gauss_seidel(
		a: Array2<MyF>, b: Array1<MyF>, mut x0: Array1<MyF>, threshold: MyF, max_step: u16,
	) -> Array1<MyF> {
		let n = b.dim();
	
		// println!("Ax=b :\n{} * {} = {}", a, x0, b);
		let mut x = Array1::<MyF>::zeros(n);
		let mut step: u16 = 0;
		loop {
			step += 1;
	
			for i in 0..n {
				x[i] = if a[[i, i]] == 0. { 1. } else { 1. / a[[i, i]] }
					* (b[i]
						- (0..i).map(|j| a[[i, j]] * x[j]).sum::<MyF>()
						- ((i + 1)..n).map(|j| a[[i, j]] * x0[j]).sum::<MyF>())
			}
	
			let difference = &a.dot(&x) - &b;
			let residue: MyF = difference.iter().map(|x| x.powf(2.)).sum();
			if residue < threshold || step >= max_step {
				break x;
			}
			x0 = x.clone();
		}
	}
}
