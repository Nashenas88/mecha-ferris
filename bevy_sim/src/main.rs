///! MechaFerris Simulator
///!
///! Based on cu_rp_balancebot: https://github.com/copper-project/copper-rs/tree/master/examples/cu_rp_balancebot
mod world;

use avian3d::prelude::*;
use bevy::app::App;
use bevy::prelude::*;
use bevy::window::{Window, WindowPlugin};
use bevy::DefaultPlugins;
use kinematics::walking::{MechaLeg, VisitLeg};
use kinematics::{DefaultConsts, ExpensiveMath};

use crate::world::{Base, Femur, Tibia};
const NUM_LEGS: usize = 6;

fn main() {
    let mut world = App::new();
    world.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "MechaFerris Simulator".into(),
            ..Default::default()
        }),
        ..Default::default()
    }));

    let world = world::build_world(&mut world);
    world.add_systems(Startup, setup_kinematics);
    world.add_systems(Update, update_kinematics);
    world.run();
}

#[derive(Resource)]
struct LegVisitor {
    legs: [MechaLeg<f32, StdMath, DefaultConsts>; NUM_LEGS],
}

struct StdMath;
impl ExpensiveMath<f32> for StdMath {
    #[inline]
    fn atan2(l: f32, r: f32) -> f32 {
        l.atan2(r)
    }

    #[inline]
    fn acos(f: f32) -> f32 {
        f.acos()
    }

    #[inline]
    fn sin(f: f32) -> f32 {
        f.sin()
    }

    #[inline]
    fn cos(f: f32) -> f32 {
        f.cos()
    }

    #[inline]
    fn sincos(f: f32) -> (f32, f32) {
        (f.sin(), f.cos())
    }

    #[inline]
    fn sqrt(f: f32) -> f32 {
        f.sqrt()
    }
}

impl VisitLeg<f32, StdMath, DefaultConsts> for LegVisitor {
    fn before(
        &mut self,
        target: kinematics::Point3<f32>,
        leg: &MechaLeg<f32, StdMath, DefaultConsts>,
    ) {
        todo!()
    }

    fn on_error(
        &mut self,
        leg: &MechaLeg<f32, StdMath, DefaultConsts>,
        error: kinematics::LegError<f32>,
    ) {
        todo!()
    }

    fn after(
        &mut self,
        target: kinematics::Point3<f32>,
        leg: &MechaLeg<f32, StdMath, DefaultConsts>,
    ) {
        todo!()
    }

    fn position_start(&mut self) {
        todo!()
    }

    fn position_end(&mut self) {
        todo!()
    }

    fn servo_start(&mut self) {
        todo!()
    }

    fn servo_end(&mut self) {
        todo!()
    }
}

fn setup_kinematics(mut commands: Commands) {
    todo!()
}

fn update_kinematics(
    mut query_set: ParamSet<(
        Query<(&mut Transform, &mut ExternalForce), With<Base>>,
        Query<&Transform, With<Femur>>,
        Query<&Transform, With<Tibia>>,
    )>,
    physics_time: Res<Time<Physics>>,
    mut kinematics_ctx: ResMut<LegVisitor>,
) {
    todo!()
}
