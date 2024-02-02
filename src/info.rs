use std::time::Duration;
use eframe::egui::plot::{Corner, Legend, Line, Plot};
use eframe::egui::{Slider, Ui};
use crate::simulation::{MyF, Simulation};
use crate::utils::vec_last;


pub const INFO_SHOWN_TIME: Duration = Duration::from_secs(2);


pub struct Info {
	time: Vec<MyF>,
	kinetic_energy: Vec<MyF>,
	potential_energy: Vec<MyF>,
	total_energy: Vec<MyF>,
	shown_time: Duration,
}

impl Info {
	pub fn new() -> Self {
		Self {
			time: Vec::new(),
			kinetic_energy: Vec::new(),
			potential_energy: Vec::new(),
			total_energy: Vec::new(),
			shown_time: INFO_SHOWN_TIME,
		}
	}
	
	pub fn clear(&mut self) {
		self.time.clear();
		self.kinetic_energy.clear();
		self.potential_energy.clear();
		self.total_energy.clear();
	}
	
	pub fn update(&mut self, simulation: &Simulation) {
		self.time.push(simulation.get_time().as_secs_f64());
		let kinetic_energy = simulation.kinetic_energy();
		let potential_energy = simulation.potential_energy();
		self.kinetic_energy.push(kinetic_energy);
		self.potential_energy.push(potential_energy);
		self.total_energy.push(kinetic_energy + potential_energy);
	}
	
	pub fn show(&mut self, ui: &mut Ui) {
		let mut shown_time = self.shown_time.as_secs_f64();
		ui.add(Slider::new(&mut shown_time, 0.1..=10.).logarithmic(true)); // .show_value(false)
		self.shown_time = Duration::from_secs_f64(shown_time);
		
		let min_time = if let Some(t) = self.time.last() { t - self.shown_time.as_secs_f64() } else { 0. };
		let shown_time: Vec<MyF> = self.time.clone().into_iter().filter(|&t| t > min_time).collect();
		
		//println!("{:?}", shown_time);
		let kinetic: Vec<[MyF; 2]> = shown_time.iter()
		                                       .zip(vec_last(self.kinetic_energy.clone(), shown_time.len()))
		                                       .map(|(&t, v)| { [t, v] }).collect();
		let potential: Vec<[MyF; 2]> = shown_time.iter()
		                                         .zip(vec_last(self.potential_energy.clone(), shown_time.len()))
		                                         .map(|(&t, v)| { [t, v] }).collect();
		let total: Vec<[MyF; 2]> = shown_time.iter()
		                                     .zip(vec_last(self.total_energy.clone(), shown_time.len()))
		                                     .map(|(&t, v)| { [t, v] }).collect();
		
		Plot::new("Energy plot")
			.height(200.)
			.auto_bounds_y()
			.allow_drag(false)
			.allow_zoom(false)
			.allow_scroll(false)
			// .data_aspect(0.1)
			.legend(Legend::default().position(Corner::RightTop).background_alpha(0.))
			.allow_boxed_zoom(false)
			.show(ui, |plot_ui| {
				plot_ui.line(Line::new(kinetic).name("Kinetic"));
				plot_ui.line(Line::new(potential).name("Potential"));
				plot_ui.line(Line::new(total).name("Total"));
			});
		
		/*
		Plot::new("Energy plot 2")
			.height(200.)
			.legend(Legend::default().position(Corner::RightTop).background_alpha(0.))
			.allow_boxed_zoom(false)
			.show(ui, |plot_ui| {
				plot_ui.line(Line::new(self.time.iter()
				                           .zip(self.kinetic_energy.clone())
				                           .map(|(&t, v)| { [t, v] }).collect::<Vec<[MyF; 2]>>()).name("Kinetic"));
				plot_ui.line(Line::new(self.time.iter()
				                           .zip(self.potential_energy.clone())
				                           .map(|(&t, v)| { [t, v] }).collect::<Vec<[MyF; 2]>>()).name("Potential"));
				plot_ui.line(Line::new(self.time.iter()
				                           .zip(self.total_energy.clone())
				                           .map(|(&t, v)| { [t, v] }).collect::<Vec<[MyF; 2]>>()).name("Total"));
			});
		*/
	}
}

