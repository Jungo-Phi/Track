// TODO: Enregistrer les paramètres, les simulations
// TODO: Interaction avec la simulation
// TODO: Trouver ce qui fait augmenter l'énergie (petit DT fait mieux)
// TODO: Les contraintes (pivot)


mod physics_app;
mod utils;
mod camera;
mod particle;
mod simulation;
mod force_generators;
mod constrains;
mod rect;
mod info;

use eframe::egui::{Color32, Vec2};
use eframe::Theme;
use nalgebra::{Point2, Vector2};
use physics_app::PhysicsApp;
use crate::constrains::{Constrain, LengthConstraint, LineConstraint};
use crate::force_generators::{ForceGenerator, Constant, Spring};
use crate::particle::Particle;
use crate::rect::MyRect;
use crate::simulation::SimulationCondition;
use crate::utils::GRAVITY;



fn main() -> Result<(), eframe::Error> {
	let simulation_conditions = vec![
		SimulationCondition::new(
			"Spring mass".to_string(),
			vec![
				Particle::new(1., Point2::origin(), Vector2::zeros(), Color32::GRAY),
				Particle::new(1., Point2::new(1.5, 0.), Vector2::zeros(), Color32::RED),
			],
			vec![
				Box::new(LineConstraint::new(0, Vector2::new(1., 0.))) as Box<dyn Constrain + Send>,
				Box::new(LineConstraint::new(0, Vector2::new(0., 1.))),
				Box::new(LineConstraint::new(1, Vector2::new(1., 0.))),
			],
			vec![
				Box::new(Spring::new(0, 1, 30., 0., 1.)) as Box<dyn ForceGenerator + Send>,
			],
			None,
		),
		SimulationCondition::new(
			"3 Springs".to_string(),
			vec![
				Particle::new(1., Point2::new(-0.3, 1.5), Vector2::new(1., 0.), Color32::RED),
				Particle::new(1., Point2::new(0.6, 1.), Vector2::new(0., 0.), Color32::BLUE),
				Particle::new(1., Point2::new(0.4, 1.3), Vector2::new(0., 0.), Color32::GREEN),
			],
			vec![],
			vec![
				Box::new(Constant::new(Vector2::new(0., -GRAVITY))) as Box<dyn ForceGenerator + Send>,
				Box::new(Spring::new(0, 1, 30., 0.2, 1.)),
				Box::new(Spring::new(1, 2, 20., 0.2, 0.3)),
			],
			Some(MyRect::new(Point2::new(-1., 0.), Vector2::new(2., 2.))),
		),
		SimulationCondition::new(
			"Pendulum".to_string(),
			vec![
				Particle::new(1., Point2::origin(), Vector2::zeros(), Color32::BLUE),
				Particle::new(1., Point2::new(1., 0.), Vector2::zeros(), Color32::RED),
			],
			vec![
				Box::new(LineConstraint::new(0, Vector2::new(1., 0.))) as Box<dyn Constrain + Send>,
				// Box::new(LineConstraint::new(0, Vector2::new(0., 1.))),
				Box::new(LengthConstraint::new(0, 1, 6.)),
			],
			vec![
				Box::new(Constant::new(Vector2::new(0., -GRAVITY))) as Box<dyn ForceGenerator + Send>,
			],
			Some(MyRect::new(Point2::new(-2., -2.), Vector2::new(4., 4.)))
		)
	];
	
	
	let options = eframe::NativeOptions {
		decorated: true,
		icon_data: None,
		initial_window_size: Some(Vec2::new(1000., 600.)),
		min_window_size: Some(Vec2::new(750., 400.)),
		centered: true,
		resizable: true,
		default_theme: Theme::Light,
		..Default::default()
	};
	
	eframe::run_native(
		"Arnaud's 2D physics engine",
		options,
		Box::new(|cc| Box::new(PhysicsApp::new(cc, simulation_conditions)))
	)
}
