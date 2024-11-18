use avian3d::prelude::*;
use bevy::core_pipeline::fxaa::Fxaa;
use bevy::core_pipeline::Skybox;
use bevy::input::keyboard::KeyCode;
use bevy::input::mouse::{MouseButton, MouseMotion, MouseWheel};
use bevy::pbr::{
    DefaultOpaqueRendererMethod, ScreenSpaceReflectionsBundle, ScreenSpaceReflectionsSettings,
};
use bevy::prelude::*;
use bevy_mod_picking::prelude::*;
use cached_path::{Cache, ProgressBar};
use std::path::Path;
use std::{fs, io};

#[derive(Resource)]
struct CameraControl {
    rotate_sensitivity: f32,
    zoom_sensitivity: f32,
    move_sensitivity: f32,
}

#[derive(Resource, PartialEq, Eq)]
enum SimulationState {
    Running,
    Paused,
}

#[derive(Component)]
pub struct Base;

#[derive(Component)]
pub struct Femur;

#[derive(Component)]
pub struct Tibia;

pub fn build_world(app: &mut App) -> &mut App {
    app.insert_resource(Msaa::Off)
        .add_plugins((
            DefaultPickingPlugins,
            PhysicsPlugins::default().with_length_unit(1000.0),
            // EditorPlugin::default(),
        ))
        .insert_resource(DefaultOpaqueRendererMethod::default())
        .insert_resource(SimulationState::Running)
        .insert_resource(CameraControl {
            rotate_sensitivity: 0.05,
            zoom_sensitivity: 3.5,
            move_sensitivity: 0.01,
        })
        .insert_resource(Gravity::default())
        .insert_resource(Time::<Physics>::default())
        .add_systems(Startup, setup_scene)
        .add_systems(Startup, setup_ui)
        .add_systems(Update, setup_entities)
        .add_systems(Update, toggle_simulation_state)
        .add_systems(Update, camera_control_system)
        .add_systems(Update, update_physics)
        // .add_systems(Update, global_body_drag_listener)
        .add_systems(PostUpdate, reset_sim)
}

fn setup_scene(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    todo!()
}

fn setup_ui(mut commands: Commands) {
    todo!()
}

fn setup_entities(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    query: Query<(Entity, &Name)>,
) {
    todo!()
}

fn toggle_simulation_state(
    mut state: ResMut<SimulationState>,
    keyboard_inout: Res<ButtonInput<KeyCode>>,
) {
    todo!()
}

fn camera_control_system(
    control: Res<CameraControl>,
    keys: Res<ButtonInput<KeyCode>>,
    mut scroll_evr: EventReader<MouseWheel>,
    mut mouse_motion_evr: EventReader<MouseMotion>,
    mut query: Query<&mut Transform, With<Camera>>,
    time: Res<Time>,
    mouse_button_input: Res<ButtonInput<MouseButton>>,
) {
    todo!()
}

fn update_physics(state: Res<SimulationState>, mut time: ResMut<Time<Physics>>) {
    if *state == SimulationState::Paused {
        time.pause();
    } else {
        time.unpause()
    }
}

fn reset_sim(
    keys: Res<ButtonInput<KeyCode>>,
    mut query: Query<(
        Option<&Base>,
        Option<&Femur>,
        Option<&Tibia>,
        Option<&mut Transform>, // ensure the transform is mutable
        Option<&mut ExternalForce>,
    )>,
) {
    todo!()
}
