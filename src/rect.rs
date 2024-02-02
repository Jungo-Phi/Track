#![allow(unused)]
use nalgebra::{ArrayStorage, Const, Matrix, Matrix2, Point2, Vector2};
use crate::simulation::MyF;


#[derive(Copy, Clone)]
pub struct MyRect {
	position: Point2<MyF>,
	size: Vector2<MyF>,
}

impl MyRect {
	pub fn new(position: Point2<MyF>, size: Vector2<MyF>) -> Self { Self { position, size } }
	
	pub fn width(&self) -> MyF { self.size.x }
	pub fn height(&self) -> MyF { self.size.y }
	
	pub fn left(&self) -> MyF { self.position.x }
	pub fn right(&self) -> MyF { self.position.x + self.size.x }
	pub fn bottom(&self) -> MyF { self.position.y }
	pub fn top(&self) -> MyF { self.position.y + self.size.y }
	
	pub fn bottom_left(&self) -> Point2<MyF> { self.position }
	pub fn bottom_right(&self) -> Point2<MyF> { self.position + Vector2::new(self.width(), 0.) }
	pub fn top_left(&self) -> Point2<MyF> { self.position + Vector2::new(0., self.height()) }
	pub fn top_right(&self) -> Point2<MyF> { self.position + self.size }
	
	pub fn center(&self) -> Point2<MyF> { self.position + 0.5 * self.size }
	
	/// Returns a smaller rect cropped by the same value in all sides
	pub fn cropped(&self, value: MyF) -> Self {
		let vec_value = Vector2::new(value, value);
		Self { position: self.position + vec_value, size: self.size - 2. * vec_value }
	}
}