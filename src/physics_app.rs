use std::ops::RangeInclusive;
use std::time::Duration;
use eframe::{App, CreationContext, egui};
use eframe::egui::{Color32, Sense, Rounding, Key, Slider, ComboBox, WidgetText, Ui, Context};

use crate::camera::Camera;
use crate::info::Info;
use crate::simulation::{MyF, Simulation, SimulationCondition};
use crate::utils::{scale2unit, THIN_BLACK_STROKE, to_point2, to_vector2};



pub const DEFAULT_DELTA: Duration = Duration::from_millis(2);


/// Physics App is an app to work with Arnaud's 2D physics engine.
///
/// The engine itself is in "Simulation".
///
/// This struct is for the UI to control the simulation and it's look.
pub struct PhysicsApp {
	show_forces: bool,
	show_speeds: bool,
	unit_scale: i16,
	background_color: Color32,
	bounding_box_color: Color32,
	camera: Camera,
	simulation_speed: f32,
	delta: Duration,
	real_time: Duration,
	playing: bool,
	simulation_conditions: Vec<SimulationCondition>,
	simulation_id: usize,
	simulation: Simulation,
	info: Info,
}

impl PhysicsApp {
	pub fn new(_cc: &CreationContext, simulation_conditions: Vec<SimulationCondition>) -> Self {
		let cond = simulation_conditions[0].clone();
		let simulation = Simulation::new(cond);
		Self {
			show_forces: true,
			show_speeds: true,
			unit_scale: 0, // meters ( utils::scale2unit() )
			background_color: Color32::LIGHT_GRAY,
			bounding_box_color: Color32::LIGHT_GRAY.linear_multiply(0.8),
			camera: Camera::new(100.,
			                    RangeInclusive::new(20., 1000.),
			                    RangeInclusive::new(-40., 40.),
			                    RangeInclusive::new(-30., 30.)),
			simulation_speed: 1.,
			delta: DEFAULT_DELTA,
			real_time: Duration::ZERO,
			playing: false,
			simulation_conditions,
			simulation_id: 0,
			simulation,
			info: Info::new(),
		}
	}
}


impl PhysicsApp {
	fn reset(&mut self) {
		self.playing = false;
		self.real_time = Duration::ZERO;
		self.simulation.reset();
		self.info.clear();
	}
	
	fn menus(&mut self, _ctx: &Context, ui: &mut Ui) {
		ui.menu_button("File", |ui| {
			ui.menu_button("Simulations", |ui| {
				(0..self.simulation_conditions.len()).for_each(|i| {
					if ui.selectable_value(&mut self.simulation_id, i, self.simulation_conditions[i].get_name()).clicked() {
						self.simulation.change_conditions(self.simulation_conditions[i].clone());
						self.reset();
						ui.close_menu();
					};
				});
			});
			if ui.button("Exit").clicked() {
				std::process::exit(0);
			}
		});
	}
	
	fn parameters_panel(&mut self, ctx: &Context, ui: &mut Ui) {
		ui.vertical_centered(|ui| {
			ui.heading("Parameters");
			ui.add_space(5.);
		});
		
		ui.horizontal(|ui| {
			let id_source = ui.label("Grid unit").id;
			ComboBox::from_id_source(id_source)
				.selected_text(WidgetText::from(scale2unit(self.unit_scale)))
				.width(40.)
				.show_ui(ui, |ui| {
					ui.selectable_value(&mut self.unit_scale, -3, "km");
					ui.selectable_value(&mut self.unit_scale, 0, "m");
					ui.selectable_value(&mut self.unit_scale, 1, "dm");
					ui.selectable_value(&mut self.unit_scale, 2, "cm");
					ui.selectable_value(&mut self.unit_scale, 3, "mm");
					ui.selectable_value(&mut self.unit_scale, 4, "μm");
				});
		});
		ui.checkbox(&mut self.show_forces, "Forces");
		ui.checkbox(&mut self.show_speeds, "Speeds");
		ui.horizontal(|ui| {
			ui.label("Background color");
			ui.color_edit_button_srgba(&mut self.background_color);
		});
		ui.horizontal(|ui| {
			ui.label("Bounding box color");
			ui.color_edit_button_srgba(&mut self.bounding_box_color);
		});
		
		egui::containers::Frame::canvas(&ctx.style()).show(ui, |ui| {
			ui.label("Canvas");
		});
	}
	
	fn info_panel(&mut self, ctx: &Context, ui: &mut Ui) {
		ui.vertical_centered(|ui| {
			ui.heading("Simulation info");
			ui.add_space(5.);
		});
		
		ui.horizontal(|ui| {
			let fps = 1. / ctx.input(|input| input.stable_dt);
			ui.strong("FPS:");
			ui.monospace(format!("{:.2}", fps));
		});
		
		ui.horizontal(|ui| {
			ui.strong("Time:");
			ui.monospace(format!("{:.2}", self.simulation.get_time().as_secs_f64()));
		});
		
		ui.separator();
		
		ui.horizontal(|ui| {
			let kinetic_energy = self.simulation.kinetic_energy();
			let potential_energy = self.simulation.potential_energy();
			let total_energy = kinetic_energy + potential_energy;
			ui.strong("Energy:");
			ui.monospace(format!("{:.3}", total_energy));
		});
		
		self.info.show(ui);
	}
	
	fn control_panel(&mut self, ctx: &Context, ui: &mut Ui) {
		ui.vertical_centered(|ui| {
			ui.horizontal(|ui| {
				if ui.button("⟳").on_hover_text("reset speed").clicked() { self.simulation_speed = 1. };
				ui.add(Slider::new(&mut self.simulation_speed, 0.1..=10.).logarithmic(true)); // .show_value(false)
				
				if ui.button("⟳").on_hover_text("reset dt").clicked() {
					self.delta = DEFAULT_DELTA;
				};
				let mut delta_millis = self.delta.as_micros() as f64 / 1000.;
				ui.add(Slider::new(&mut delta_millis, 0.1..=10.).logarithmic(true).suffix("m sec")); // .show_value
				// (false)
				self.delta = Duration::from_micros((delta_millis * 1000.) as u64);
				
				ui.group(|ui| {
					let time_zero = self.simulation.get_time().is_zero();
					let mut k = time_zero;
					ui.selectable_value(&mut k, true, "⟳");
					if k && !time_zero || ctx.input(|input| input.key_pressed(Key::R)) {
						self.reset();
					}
					// Send PLAY
					let mut playing = self.playing;
					
					k |= playing;
					ui.selectable_value(&mut k, false, "⏸");
					playing &= k && !time_zero;
					ui.selectable_value(&mut playing, true, "▶");
					
					if ctx.input(|input| input.key_pressed(Key::Space)) {
						playing = !playing;
					};
					self.playing = playing;
				});
			});
		});
	}
	
	fn simulation_panel(&mut self, ctx: &Context, ui: &mut Ui) {
		let (response, mut painter) = ui.allocate_painter(
			ui.available_size(),
			Sense::drag(),
		);
		
		self.camera.resize(response.rect);
		
		if let Some(hover_pos) = response.hover_pos() {
			self.camera.translate(to_vector2(response.drag_delta() + ctx.input(|i| i.scroll_delta)));
			
			let scaling = ctx.input(|i| i.zoom_delta()) as MyF;
			let center = to_point2(hover_pos - self.camera.origin()).coords;
			self.camera.change_scale(scaling, center);
		}
		/*
		if input.mouse.left_button.is_pressed() {
			let mouse_position = input.mouse.position;
			for (index, particle) in self.particles.iter().enumerate() {
				if particle.collide_point(self.camera.transform.inverse() * mouse_position.cast()) {
					self.mouse_spring.set_end2_index(index);
					break;
				}
			}
		} else if input.mouse.left_button.is_released() {
			self.mouse_spring.set_end2_index(0);
		}
		 */
		painter.rect_filled(response.rect, Rounding::none(), self.background_color);
		
		self.camera.draw_grid(&mut painter, true, self.unit_scale);
		self.simulation.draw(&mut painter, &self.camera, self.show_forces, self.show_speeds);
		
		painter.rect_stroke(response.rect, Rounding::none(), THIN_BLACK_STROKE);
		
		if self.playing {
			ctx.request_repaint(); // To update every frame
		}
	}
	
	fn bottom_panel(&mut self, _ctx: &Context, ui: &mut Ui) {
		ui.label("bottom panel");
	}
}


impl App for PhysicsApp {
	fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
		// Basically the layout of the panels
		egui::CentralPanel::default().show(ctx, |ui| {
			egui::TopBottomPanel::top("top_panel")
				.show_inside(ui, |ui| self.menus(ctx, ui));
			
			egui::TopBottomPanel::bottom("bottom_panel")
				.resizable(false)
				.default_height(20.0)
				.show_inside(ui, |ui| self.bottom_panel(ctx, ui));
			
			egui::SidePanel::left("left_panel")
				.resizable(false)
				.default_width(120.0)
				.show_inside(ui, |ui| self.parameters_panel(ctx, ui));
			
			egui::SidePanel::right("right_panel")
				.resizable(true)
				.default_width(225.0)
				.width_range(150.0..=300.0)
				.show_inside(ui, |ui| self.info_panel(ctx, ui));
			
			egui::CentralPanel::default().show_inside(ui, |ui| {
				ui.vertical_centered(|ui| {
					self.control_panel(ctx, ui);
				});
				self.simulation_panel(ctx, ui);
			});
		});
		
		if self.playing {
			let t = Duration::from_secs_f32(ctx.input(|input| input.unstable_dt)).mul_f32(self.simulation_speed);
			self.real_time += t;
			//println!("{:?}", t);
			loop {
				if self.simulation.get_time() > &self.real_time { break }
				self.simulation.update(self.delta);
				self.info.update(&self.simulation);
			}
		}
	}
}
