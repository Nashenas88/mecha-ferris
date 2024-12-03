//! MechaFerris Simulator
//!
//! Based on cu_rp_balancebot: https://github.com/copper-project/copper-rs/tree/master/examples/cu_rp_balancebot
mod world;

use avian3d::prelude::*;
use bevy::app::App;
use bevy::prelude::*;
use bevy::window::{Window, WindowPlugin};
use bevy::DefaultPlugins;
use kinematics::walking::{MechaLeg, VisitLeg};
use kinematics::{DefaultConsts, ExpensiveMath, Leg};

use crate::world::{Base, Femur, Tibia};
const NUM_LEGS: usize = 6;
const NUM_SERVOS_PER_LEG: usize = 3;

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

pub struct Joint {
    servo: u8,
    // /// Calibration for when the joint is running as part of the robot.
    // cal: CalData,
    // /// Calibration for when the joint is being calibrated.
    // calibrating: CalData,
}

#[derive(Resource)]
struct LegVisitor {
    legs: [[Joint; NUM_SERVOS_PER_LEG]; NUM_LEGS],
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
    }

    fn on_error(
        &mut self,
        leg: &MechaLeg<f32, StdMath, DefaultConsts>,
        error: kinematics::LegError<f32>,
    ) {
        tracing::error!("Leg error: {error:?}");
    }

    fn after(
        &mut self,
        target: kinematics::Point3<f32>,
        leg: &MechaLeg<f32, StdMath, DefaultConsts>,
    ) {
    }

    fn position_start(&mut self) {}

    fn position_end(&mut self) {}

    fn servo_start(&mut self) {}

    fn servo_end(&mut self) {}
}

fn setup_kinematics(mut commands: Commands) {
    // todo!()
    let leg_visitor = LegVisitor {
        legs: [
            [Joint { servo: 0 }, Joint { servo: 1 }, Joint { servo: 2 }],
            [Joint { servo: 3 }, Joint { servo: 4 }, Joint { servo: 5 }],
            [Joint { servo: 6 }, Joint { servo: 7 }, Joint { servo: 8 }],
            [Joint { servo: 9 }, Joint { servo: 10 }, Joint { servo: 11 }],
            [
                Joint { servo: 12 },
                Joint { servo: 13 },
                Joint { servo: 12 },
            ],
            [
                Joint { servo: 15 },
                Joint { servo: 16 },
                Joint { servo: 17 },
            ],
        ],
    };
    commands.insert_resource(leg_visitor);
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
    // todo!()
}
