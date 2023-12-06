use core::hint::unreachable_unchecked;

use crate::game::StdMath;
use crate::servo::Servo;

use gdnative::api::*;
use gdnative::prelude::*;
use kinematics::walking::{MechaLeg, VisitLeg, Walking};
use kinematics::{DefaultConsts, LegConsts, LegError, Point3};
use state::RobotState;
use state::StateMachine;

const BODY_RADIUS: f32 = 75.0;
const DEFAULT_RADIUS: f32 = BODY_RADIUS + DefaultConsts::COXA_LENGTH + 20.0;
const DEFAULT_HEIGHT: f32 = 90.0;
// const HEIGHT_MOVEMENT: f64 = 10.0;
const NUM_LEGS: usize = 6;
const NUM_SERVOS_PER_LEG: usize = 3;

fn servo_name<T, C>(leg: &MechaLeg<f32, T, C>) -> &'static str {
    match leg.idx() {
        0 => "Body/Spatial1/CoxaServo",
        1 => "Body/Spatial2/CoxaServo",
        2 => "Body/Spatial3/CoxaServo",
        3 => "Body/Spatial4/CoxaServo",
        4 => "Body/Spatial5/CoxaServo",
        5 => "Body/Spatial6/CoxaServo",
        _ => unsafe { unreachable_unchecked() },
    }
}

fn servo_base(idx: u8) -> &'static str {
    match idx {
        0 => "Body/Spatial1",
        1 => "Body/Spatial2",
        2 => "Body/Spatial3",
        3 => "Body/Spatial4",
        4 => "Body/Spatial5",
        5 => "Body/Spatial6",
        _ => unsafe { unreachable_unchecked() },
    }
}

/// The Walker "class"
#[derive(NativeClass)]
#[inherit(Spatial)]
#[register_with(Self::register_builder)]
pub struct Walker {
    time: f64,
    height_time: f64,
    step_delay: f64,
    #[property]
    anim_scale: f64,
    walking: Walking<StdMath, DefaultConsts>,
    #[property]
    base_radius: f64,
    #[property]
    base_height: f64,
    #[property]
    walking_radius: f64,
    reset: bool,
    #[property]
    target: NodePath,
    #[property]
    target_coxa_space: NodePath,
    #[property]
    target_femur_space: NodePath,
    robot_state: RobotState<(), NUM_SERVOS_PER_LEG, NUM_LEGS>,
    tracking_nodes: Vec<Ref<CSGSphere>>,
}

const FREQUENCY: f64 = 2.0;
const HEIGHT_VARIATION: f64 = 20.0;

#[methods]
impl Walker {
    // Register the builder for methods, properties and/or signals.
    fn register_builder(_builder: &ClassBuilder<Self>) {
        godot_print!("Walker builder is registered!");
    }

    /// The "constructor" of the class.
    fn new(_owner: &Spatial) -> Self {
        let mut state = RobotState::new([[(); NUM_SERVOS_PER_LEG]; NUM_LEGS]);
        state.leg_radius = DEFAULT_RADIUS * 1.3;
        state.body_translation.y = DEFAULT_HEIGHT;
        state.state_machine = StateMachine::Homing;
        Walker {
            time: 0.0,
            height_time: 0.0,
            step_delay: 0.0,
            anim_scale: 1.0,
            walking: Walking::new(DEFAULT_RADIUS),
            walking_radius: (DEFAULT_RADIUS * 1.3) as f64,
            base_radius: DEFAULT_RADIUS as f64,
            base_height: DEFAULT_HEIGHT as f64,
            reset: false,
            target: NodePath::default(),
            target_coxa_space: NodePath::default(),
            target_femur_space: NodePath::default(),
            robot_state: state,
            tracking_nodes: vec![],
        }
    }

    #[method]
    unsafe fn _ready(&mut self, #[base] owner: &Spatial) {
        self.set_body_height(owner, self.base_height);
        self.set_body_radius(owner, self.base_radius);
        self.step_delay = self.walking.duration() * self.anim_scale;
    }

    // This function will be called in every frame
    #[method]
    unsafe fn _process(&mut self, #[base] owner: &Spatial, delta: f64) {
        let input = Input::godot_singleton();
        if input.is_action_just_pressed("change_space", true) {
            for node in self.tracking_nodes.drain(..) {
                owner.remove_child(node);
                node.assume_unique().queue_free();
            }
            self.reset = false;
            self.walking.next_animation();
        }

        self.time += delta;
        self.height_time += delta;

        match self.robot_state.state_machine {
            StateMachine::Homing => self.home_process(owner),
            StateMachine::Looping => self.follow_process(owner),
            _ => {}
        }
    }

    fn set_body_height(&mut self, owner: &Spatial, height: f64) {
        self.robot_state.body_translation.y = height as f32;
        let node: Ref<Node> = unsafe { owner.upcast::<Node>().assume_shared() };
        let safe_node = unsafe { node.assume_safe() };
        let safe_node = safe_node.get_node("Body").unwrap();
        let body = body_from_node(&safe_node);
        body.set_translation(Vector3 {
            x: 0.0,
            y: height as f32,
            z: 0.0,
        });
    }

    fn set_body_radius(&mut self, owner: &Spatial, radius: f64) {
        self.walking.set_body_radius(radius as f32);
        let node: Ref<Node> = unsafe { owner.upcast::<Node>().assume_shared() };
        let safe_node = unsafe { node.assume_safe() };
        let safe_node = safe_node.get_node("Body").unwrap();
        let body = body_from_node(&safe_node);
        body.set_radius(radius);

        for i in 0..NUM_LEGS {
            let node: Ref<Node> = unsafe { owner.upcast::<Node>().assume_shared() };
            let safe_node = unsafe { node.assume_safe() };
            let safe_node = safe_node.get_node(servo_base(i as u8)).unwrap();
            let node = unsafe { safe_node.assume_safe() };
            let spatial = node.cast::<Spatial>().unwrap();

            let angle = i as f32 * std::f32::consts::FRAC_PI_3;
            spatial.set_translation(Vector3::new(
                radius as f32 * angle.sin(),
                0.0,
                radius as f32 * angle.cos(),
            ));
        }
    }

    fn home_process(&mut self, owner: &Spatial) {
        if self.time > 2.0 {
            self.time = 0.0;
            self.robot_state.state_machine = StateMachine::Looping;
            return;
        }

        let mut homing_visitor = HomingVisitor { owner };
        self.walking.home(
            &self.robot_state,
            &mut homing_visitor,
            self.time as f32 / 2.0 * core::f32::consts::TAU,
        );
    }

    unsafe fn follow_process(&mut self, owner: &Spatial) {
        if self.time >= self.step_delay {
            self.time %= self.step_delay;
            let (reset, animation_duration) = self.walking.next_animation();
            self.reset = !reset;
            if !self.reset {
                for node in self.tracking_nodes.drain(..) {
                    owner.remove_child(node);
                    node.assume_unique().queue_free();
                }
            }
            self.step_delay = animation_duration * self.anim_scale;
        }

        let next_height = self.base_height
            - HEIGHT_VARIATION
                * (self.height_time * FREQUENCY * core::f64::consts::FRAC_PI_2).sin();
        godot_print!("Next height is {}", next_height);
        self.set_body_height(owner, next_height);

        // Figure out how far into the animation we need to render.
        let interpolation = self.time / self.step_delay;
        let mut walker_visitor = WalkerVisitor {
            owner,
            reset: self.reset,
            tracking_nodes: &mut self.tracking_nodes,
        };
        self.walking
            .walk(&self.robot_state, &mut walker_visitor, interpolation as f32);
    }

    unsafe fn move_leg(owner: &Spatial, mecha_leg: &MechaLeg<f32, StdMath, DefaultConsts>) {
        let node_name = servo_name(mecha_leg);
        let leg = &mut mecha_leg.leg();

        let deg_coxa_servo_angle = rad_to_degree(leg.coxa_servo_angle());
        let deg_femur_servo_angle = rad_to_degree(leg.femur_servo_angle());
        let deg_tibia_servo_angle = rad_to_degree(leg.tibia_servo_angle());

        let mut node: Ref<Node> = owner.upcast::<Node>().assume_shared();
        let safe_node = node.assume_safe();
        let safe_node = safe_node.get_node(node_name).unwrap();
        let servo = servo_from_node(&safe_node);
        let mut next_node = servo
            .map_mut(|servo, _cylinder| {
                servo.set_target_angle(deg_coxa_servo_angle);
                servo.next_servo()
            })
            .unwrap();

        node = servo.base().upcast::<Node>().assume_shared();
        let safe_node = node.assume_safe();
        let safe_node = safe_node.get_node(next_node.unwrap()).unwrap();
        let servo = servo_from_node(&safe_node);
        next_node = servo
            .map_mut(|servo, _cylinder| {
                servo.set_target_angle(deg_femur_servo_angle);
                servo.next_servo()
            })
            .unwrap();

        node = servo.base().upcast::<Node>().assume_shared();
        let safe_node = node.assume_safe();
        let safe_node = safe_node.get_node(next_node.unwrap()).unwrap();
        let servo = servo_from_node(&safe_node);
        servo
            .map_mut(|servo, _cylinder| {
                servo.set_target_angle(deg_tibia_servo_angle);
            })
            .unwrap();
    }
}

struct HomingVisitor<'a> {
    owner: &'a Spatial,
}

impl<'a> VisitLeg<f32, StdMath, DefaultConsts> for HomingVisitor<'a> {
    fn before(&mut self, _: Point3<f32>, _: &MechaLeg<f32, StdMath, DefaultConsts>) {
        // no-op
    }

    fn on_error(&mut self, leg: &MechaLeg<f32, StdMath, DefaultConsts>, error: LegError<f32>) {
        godot_print!(
            "Failed to calculate homing leg position for leg {}: {error:?}",
            leg.idx()
        );
    }
    fn after(&mut self, _: Point3<f32>, leg: &MechaLeg<f32, StdMath, DefaultConsts>) {
        unsafe {
            Walker::move_leg(self.owner, leg);
        }
    }

    fn position_start(&mut self) {}
    fn position_end(&mut self) {}
    fn servo_start(&mut self) {}
    fn servo_end(&mut self) {}
}

struct WalkerVisitor<'a> {
    owner: &'a Spatial,
    reset: bool,
    tracking_nodes: &'a mut Vec<Ref<CSGSphere>>,
}

impl<'a> VisitLeg<f32, StdMath, DefaultConsts> for WalkerVisitor<'a> {
    fn before(&mut self, target: Point3<f32>, _: &MechaLeg<f32, StdMath, DefaultConsts>) {
        // Render movement guides.
        if !self.reset {
            let node = CSGSphere::new();
            node.translate(Vector3 {
                x: target.x,
                y: target.y,
                z: target.z,
            });
            node.set_radius(0.5);
            let node = node.into_shared();
            self.tracking_nodes.push(node);
            self.owner.add_child(node, false);
        }
    }

    fn on_error(&mut self, leg: &MechaLeg<f32, StdMath, DefaultConsts>, error: LegError<f32>) {
        godot_print!(
            "Failed to calculate leg position for leg {}: {error:?}",
            leg.idx()
        );
    }

    fn after(&mut self, _: Point3<f32>, leg: &MechaLeg<f32, StdMath, DefaultConsts>) {
        unsafe { Walker::move_leg(self.owner, leg) }
    }

    fn position_start(&mut self) {}
    fn position_end(&mut self) {}
    fn servo_start(&mut self) {}
    fn servo_end(&mut self) {}
}

fn body_from_node(node: &Ref<Node>) -> TRef<'_, CSGCylinder> {
    unsafe {
        let node = node.assume_safe();
        node.cast::<CSGCylinder>().unwrap()
    }
}

fn servo_from_node(node: &Ref<Node>) -> TInstance<'_, Servo> {
    unsafe {
        let node = node.assume_safe();
        let node = node.cast::<CSGCylinder>().unwrap();
        node.cast_instance::<Servo>().unwrap()
    }
}

fn rad_to_degree(rad: f32) -> f32 {
    rad / core::f32::consts::PI * 180.0
}
