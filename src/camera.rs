#![allow(unused)]
use std::vec::Vec;
use std::ops::RangeInclusive;
use nalgebra::{Point2, Similarity2, Translation2, Vector2};

use eframe::egui::{Color32, Rounding, Rect, Painter, Response, Align, FontId, Shape, Align2};
use eframe::egui::Shape::*;
use eframe::emath::{lerp, Pos2, Vec2};
use eframe::epaint::{CircleShape, Stroke};
use crate::rect::MyRect;

use crate::simulation::MyF;
use crate::utils::{MIRROR_Y, THIN_BLACK_STROKE, to_point2, to_pos2, to_vector2};


pub struct Camera {
	transform: Similarity2<MyF>,
	rect: Rect,
	min_scale: MyF,
	max_scale: MyF,
	h_range: RangeInclusive<MyF>,
	v_range: RangeInclusive<MyF>,
}

impl Camera {
	const GRID_SCALE: MyF = -0.5;
	const P: [f32; 4] = [0., 0.10, 0.30, 0.60];
	const Q: [f32; 4] = [0., 0.25, 0.45, f32::NAN];
	
	pub fn new(
		default_zoom: MyF, zoom_range: RangeInclusive<MyF>, h_range: RangeInclusive<MyF>, v_range: RangeInclusive<MyF>,
	) -> Self {
		Camera {
			rect: Rect { min: Pos2::ZERO, max: Pos2::ZERO },
			transform: Similarity2::new(Vector2::zeros(), 0., default_zoom),
			min_scale: *zoom_range.start(),
			max_scale: *zoom_range.end(),
			h_range,
			v_range,
		}
	}
	
	pub fn origin(&self) -> Vec2 { self.rect.min.to_vec2() }
	
	pub fn world2cam_p(&self, point: Point2<MyF>) -> Pos2 {
		to_pos2(self.transform.transform_point(&(MIRROR_Y * point))) + self.origin()
	}
	pub fn cam2world_p(&self, point: Pos2) -> Point2<MyF> {
		MIRROR_Y * (self.transform.inverse_transform_point(&to_point2(point - self.origin())))
	}
	pub fn cam2world_v(&self, vector: Vec2) -> Vector2<MyF> {
		MIRROR_Y * self.transform.inverse_transform_vector(&to_vector2(vector))
	}
	
	fn scale(&self) -> MyF {
		self.transform.scaling()
	}
	
	fn translation(&self) -> Vector2<MyF> {
		self.transform.isometry.translation.vector
	}
	
	/// Translates the camera by 'delta' while restricting it within it limits.
	pub fn translate(&mut self, delta: Vector2<MyF>) {
		if delta.is_empty() { return; }
		let resolution = to_vector2(self.rect.size());
		
		let old_translation = self.translation();
		self.transform.append_translation_mut(&Translation2::from(delta));
		
		let start = self.transform.inverse_transform_point(&Point2::origin());
		let end = self.transform.inverse_transform_point(&Point2::from(resolution));
		
		if start.x < *self.h_range.start() {
			self.transform.isometry.translation.x = -*self.h_range.start() * self.scale();
		}
		if start.y < *self.v_range.start() {
			self.transform.isometry.translation.y = -*self.v_range.start() * self.scale();
		}
		if end.x > *self.h_range.end() {
			self.transform.isometry.translation.x = -*self.h_range.end() * self.scale() + resolution.x;
		}
		if end.y > *self.v_range.end() {
			self.transform.isometry.translation.y = -*self.v_range.end() * self.scale() + resolution.y;
		}
	}
	
	/// Scales the camera by 'scaling' while restricting it within it limits.
	pub fn change_scale(&mut self, scaling: MyF, center: Vector2<MyF>) {
		if scaling == 1. {
			return;
		}
		if self.min_scale > self.scale() * scaling { // near min scale
			if self.scale() <= self.min_scale { return; }
			let adjusted_scaling = self.min_scale / self.scale();
			self.transform.append_scaling_mut(adjusted_scaling);
			self.translate((1. - adjusted_scaling) * center);
		} else if self.max_scale < self.scale() * scaling { // near max scale
			if self.scale() >= self.max_scale { return; }
			let adjusted_scaling = self.max_scale / self.scale();
			self.transform.append_scaling_mut(adjusted_scaling);
			self.translate((1. - adjusted_scaling) * center);
		} else {
			self.transform.append_scaling_mut(scaling);
			self.translate((1. - scaling) * center);
		}
	}
	pub fn resize(&mut self, new_rect: Rect) {
		self.translate(-0.5 * to_vector2(self.rect.size() - new_rect.size()));
		self.rect = new_rect;
	}
	
	/// Paint a filled rectangle in world space
	pub fn fill_rect(&self, painter: &mut Painter, rect: MyRect, rounding: Rounding, fill_color: Color32) {
		let min = self.world2cam_p(rect.top_left());
		let max = self.world2cam_p(rect.bottom_right());
		painter.rect_filled(Rect { min, max }, rounding, fill_color);
	}
	/// Paint the stroke of a rectangle in world space
	pub fn stroke_rect(&self, painter: &mut Painter, rect: MyRect, rounding: Rounding, stroke: Stroke) {
		let min = self.world2cam_p(rect.top_left());
		let max = self.world2cam_p(rect.bottom_right());
		painter.rect_stroke(Rect { min, max }, rounding, stroke);
	}
	
	/// Paint a filled circle in world space
	pub fn fill_circle(&self, painter: &mut Painter, position: Point2<MyF>, radius: MyF, fill_color: Color32) {
		let center = self.world2cam_p(position);
		let radius = (self.scale() * radius) as f32;
		painter.circle_filled(center, radius, fill_color);
	}
	/// Paint the stroke of a circle in world space
	pub fn stroke_circle(&self, painter: &mut Painter, position: Point2<MyF>, radius: MyF, stroke: Stroke) {
		let center = self.world2cam_p(position);
		let radius = (self.scale() * radius) as f32;
		painter.circle_stroke(center, radius, stroke);
	}
	/// Paint a point in world space (unlike a circle, the radius does not change as you zoom)
	pub fn paint_point(&self, painter: &mut Painter, position: Point2<MyF>, radius: f32, fill: Color32) {
		let center = self.world2cam_p(position);
		painter.circle(center, radius, fill, THIN_BLACK_STROKE);
	}
	
	/// Paint a line in world space
	pub fn paint_line(&self, painter: &mut Painter, start: Point2<MyF>, end: Point2<MyF>, stroke: Stroke) {
		let start = self.world2cam_p(start);
		let end = self.world2cam_p(end);
		painter.line_segment([start, end], stroke);
	}
	/// Paint connected lines in world space
	pub fn paint_lines(&self, painter: &mut Painter, points: Vec<Point2<MyF>>, stroke: Stroke) {
		let points = points.into_iter().map(|point| self.world2cam_p(point)).collect();
		painter.add(Shape::line(points, stroke));
	}
	/// Paint a vertical line across the height of the screen in world space
	pub fn paint_vline(&self, painter: &mut Painter, x: MyF, stroke: Stroke) {
		let x = (self.scale() * x + self.translation().x) as f32 + self.rect.left();
		painter.vline(x, self.rect.y_range(), stroke);
	}
	/// Paint a horizontal line across the width of the screen in world space
	pub fn paint_hline(&self, painter: &mut Painter, y: MyF, stroke: Stroke) {
		let y = (self.scale() * y + self.translation().y) as f32 + self.rect.top();
		painter.hline(self.rect.x_range(), y, stroke);
	}
	
	/// Paint a wire arrow in world space
	pub fn paint_wire_arrow(&self, painter: &mut Painter, start: Point2<MyF>, end: Point2<MyF>, stroke: Stroke) {
		if start == end { return; }
		let start = self.world2cam_p(start);
		let end = self.world2cam_p(end);
		painter.arrow(start, end - start, stroke);
	}
	
	/// Paint a convex polygon in world space.
	///
	/// The most performant winding order is clockwise.
	pub fn paint_polygon(&self, painter: &mut Painter, vertices: Vec<Point2<MyF>>, fill: Color32, stroke: Stroke) {
		let points: Vec<Pos2> = vertices.iter().map(|&point| self.world2cam_p(point)).collect();
		painter.add(Shape::convex_polygon(points, fill, stroke));
	}
	
	/*
	/// Draws an arrow as seen by the camera
	pub fn paint_thick_arrow(
		&self, painter: &mut Painter, color: Color32, start: Point2<MyF>, end: Point2<MyF>, width: MyF,
	) {
		if start == end {
			return;
		}
		let start = self.transform * start;
		let end = self.transform * end;
		let width = self.transform.scaling() * width;
		// TODO clean up
		let x_dir = end - start;
		let y_dir = x_dir.perpendicular() * width / 2.;
		let transform = Transform2::from_matrix_unchecked(Matrix3::new(
			x_dir.x, y_dir.x, start.x, x_dir.y, y_dir.y, start.y, 0., 0., 1.,
		));

		let head_back: MyF = 1. - 3. * width / x_dir.norm();

		let mut points = Vec::from([
			Point2::new(head_back, -1.),
			Point2::new(head_back, -3.),
			Point2::new(1., 0.),
			Point2::new(head_back, 3.),
			Point2::new(head_back, 1.),
		]);
		if x_dir.norm() > 3. * width {
			points.append(&mut Vec::from([Point2::new(0., 1.), Point2::new(0., -1.)]));
		}
		points.iter_mut().for_each(|v| *v = transform * *v);
		let points_x: Vec<i16> = points.iter().map(|v| v.x as i16).collect();
		let points_y: Vec<i16> = points.iter().map(|v| v.y as i16).collect();

		DrawRenderer::filled_polygon(canvas, &points_x, &points_y, color).unwrap();
		DrawRenderer::polygon(canvas, &points_x, &points_y, Colors::BLACK).unwrap();
	}
	*/
	
	/// Draws a grid for the world space
	pub fn draw_grid(&self, painter: &mut Painter, graduations: bool, unit_scale: i16) {
		let log_scale = self.scale().log10() + Self::GRID_SCALE;
		let local_scale = (log_scale - log_scale.floor()) as f32; //  0=light  1=dark
		let floor_scale = MyF::powf(10., log_scale.floor());
		
		let x_transform = |x_th: i32| (self.scale() / floor_scale * x_th as MyF + self.translation().x) as f32 + self.rect.left();
		let y_transform = |y_th: i32| (-self.scale() / floor_scale * y_th as MyF + self.translation().y) as f32 + self.rect.top();
		
		// Grid
		let stroke = {
			let factor = |a: [f32; 4], i: usize| -> f32 {
				lerp(RangeInclusive::new(a[i], a[i + 1]), local_scale)
			};
			move |n_th| -> Stroke {
				let factor = if n_th % 100 == 0 {
					factor(Self::P, 2)
				} else if n_th % 50 == 0 {
					factor(Self::Q, 1)
				} else if n_th % 10 == 0 {
					factor(Self::P, 1)
				} else if n_th % 5 == 0 {
					factor(Self::Q, 0)
				} else {
					factor(Self::P, 0)
				};
				Stroke::new(1., Color32::BLACK.linear_multiply(factor))
			}
		};
		
		let transform = self.transform.inverse().append_scaling(floor_scale);
		let mut start = (transform * Point2::origin()).map(|v| v.ceil() as i32); // Top Left
		let mut end = (transform * to_point2(self.rect.size().to_pos2())).map(|v| v.ceil() as i32); // Bottom Right
		
		(start.y, end.y) = (1 - end.y, 1 - start.y);
		(start.x..end.x).for_each(|x_th| painter.vline(x_transform(x_th), self.rect.y_range(), stroke(x_th)));
		(start.y..end.y).for_each(|y_th| painter.hline(self.rect.x_range(), y_transform(y_th), stroke(y_th)));
		
		// Alignment
		let origin = self.transform * Point2::origin();
		let v_align = if origin.y <= 0. {
			Align::TOP
		} else if (origin.y as f32).lt(&self.rect.height()) { Align::Center } else { Align::BOTTOM };
		let h_align = if origin.x <= 0. {
			Align::LEFT
		} else if (origin.x as f32).lt(&self.rect.width()) { Align::Center } else { Align::RIGHT };
		
		// Axes
		let stroke = Stroke::new(1.5, Color32::BLACK.linear_multiply(0.6));
		
		let x = match h_align {
			Align::LEFT => 1.,
			Align::Center => origin.x as f32,
			Align::RIGHT => self.rect.width() - 1.,
		} + self.rect.left();
		let y = match v_align {
			Align::TOP => 1.,
			Align::Center => origin.y as f32,
			Align::BOTTOM => self.rect.height() - 1.,
		} + self.rect.top();
		painter.vline(x, self.rect.y_range(), stroke);
		painter.hline(self.rect.x_range(), y, stroke);
		
		// Graduations
		if !graduations { return; }
		let x_anchor = match h_align { Align::LEFT => Align2::LEFT_CENTER, _ => Align2::RIGHT_CENTER };
		let x_delta = match h_align { Align::LEFT => 2., _ => -2. };
		let y_anchor = match v_align { Align::BOTTOM => Align2::CENTER_BOTTOM, _ => Align2::CENTER_TOP };
		
		let kn = if local_scale < 0.2 { 20 } else if local_scale > 0.9 { 2 } else if local_scale > 0.6 { 5 } else { 10 };
		let precision = 0_i16.max(log_scale.floor() as i16 + if local_scale > 0.6 { 1 } else { 0 } - 1 - unit_scale) as usize;
		let text_scale = MyF::powf(10., unit_scale as MyF) / floor_scale;
		
		let draw_text = |n_th: i32, pos: Pos2, anchor: Align2| {
			if n_th != 0 && n_th % kn == 0 {
				painter.text(pos, anchor, format!("{:.*}", precision, n_th as MyF * text_scale),
				             FontId::proportional(12.), Color32::BLACK);
			}
		};
		(start.x..end.x).for_each(|x_th| draw_text(x_th, Pos2::new(x_transform(x_th), y), y_anchor));
		(start.y..end.y).for_each(|y_th| draw_text(y_th, Pos2::new(x + x_delta, y_transform(y_th)), x_anchor));
	}
	
	/*
	/// Draws text as seen by the camera
	pub fn draw_text(
		&self, painter: &mut Painter, text_drawer: &TextDrawer, color: Color32, position: Point2<MyF>,
		font_size: MyF, text: String, align: Align,
	) {
		if !text.is_empty() {
			let position = self.transform * position;
			let position = Point::new(position.x as i32, position.y as i32);
			let text_style = &TextStyle { font_size: (self.scale() * font_size) as u16, color, ..Default::default() };
			text_drawer.draw(canvas, position, text_style, &text, align);
		}
	}
 */
}
