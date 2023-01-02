use core::hint::unreachable_unchecked;

use crate::animation::AnimationManager;
use crate::game::StdMath;
use crate::servo::Servo;

use gdnative::api::*;
use gdnative::prelude::*;
use kinematics::Matrix4;
use kinematics::{DefaultConsts, Leg, LegConsts, Point3, Rotation3, Translation3, Vector3 as Vec3};

const BODY_RADIUS: f32 = 75.0;
const DEFAULT_RADIUS: f32 = BODY_RADIUS + <DefaultConsts as LegConsts>::COXA_LENGTH + 20.0;
const DEFAULT_HEIGHT: f32 = 90.0;
// const HEIGHT_MOVEMENT: f64 = 10.0;

#[derive(Copy, Clone, Default)]
struct MechaLeg {
    leg: Leg<f32, StdMath>,
    idx: u8,
}

impl MechaLeg {
    fn servo_name(&self) -> &'static str {
        match self.idx {
            0 => "Body/Spatial1/CoxaServo1",
            1 => "Body/Spatial2/CoxaServo2",
            2 => "Body/Spatial3/CoxaServo3",
            3 => "Body/Spatial4/CoxaServo4",
            4 => "Body/Spatial5/CoxaServo5",
            5 => "Body/Spatial6/CoxaServo6",
            _ => unsafe { unreachable_unchecked() },
        }
    }

    /// Returns matrix that can transform a point in the body space to the coxa space for a leg.
    fn origin_to_coxa(&self, height: f32, radius: f32) -> Matrix4<f32> {
        let rot = self.idx as f32 * std::f32::consts::FRAC_PI_3;
        Rotation3::from_axis_angle(&Vec3::y_axis(), std::f32::consts::FRAC_PI_2).to_homogeneous()
            * Translation3::new(0.0, -height, -radius).to_homogeneous()
            * Rotation3::from_axis_angle(&Vec3::y_axis(), -rot).to_homogeneous()
    }
}

/// The Walker "class"
#[derive(NativeClass)]
#[inherit(Spatial)]
#[register_with(Self::register_builder)]
pub struct Walker {
    time: f64,
    step_delay: f64,
    #[property]
    anim_scale: f64,
    legs: [MechaLeg; 6],
    #[property]
    base_radius: f64,
    #[property]
    base_height: f64,
    body_height: f64,
    #[property]
    walking_radius: f64,
    animation: AnimationManager,
    reset: bool,
    #[property]
    target: NodePath,
    #[property]
    target_coxa_space: NodePath,
    #[property]
    target_femur_space: NodePath,
    robot_state: RobotState,
    tracking_nodes: Vec<Ref<CSGSphere>>,
}

enum RobotState {
    Homing,
    Following,
}

#[methods]
impl Walker {
    // Register the builder for methods, properties and/or signals.
    fn register_builder(_builder: &ClassBuilder<Self>) {
        godot_print!("Walker builder is registered!");
    }

    /// The "constructor" of the class.
    fn new(_owner: &Spatial) -> Self {
        let mut legs = [MechaLeg::default(); 6];
        for (i, leg) in legs.iter_mut().enumerate() {
            leg.idx = i as u8;
        }
        Walker {
            time: 0.0,
            step_delay: 0.0,
            anim_scale: 1.0,
            legs,
            base_radius: DEFAULT_RADIUS as f64,
            base_height: DEFAULT_HEIGHT as f64,
            body_height: DEFAULT_HEIGHT as f64,
            walking_radius: (DEFAULT_RADIUS * 1.3) as f64,
            animation: AnimationManager::new(),
            reset: false,
            target: NodePath::default(),
            target_coxa_space: NodePath::default(),
            target_femur_space: NodePath::default(),
            robot_state: RobotState::Homing,
            tracking_nodes: vec![],
        }
    }

    #[method]
    unsafe fn _ready(&mut self, #[base] owner: &Spatial) {
        self.set_body_height(owner, self.base_height);
        self.set_body_radius(owner, self.base_radius);
        self.step_delay = self.animation.duration() * self.anim_scale;
    }

    // This function will be called in every frame
    #[method]
    unsafe fn _process(&mut self, #[base] owner: &Spatial, delta: f64) {
        let input = Input::godot_singleton();
        if input.is_action_just_pressed("change_space", true) {
            for node in self.tracking_nodes.drain(..) {
                owner.remove_child(node);
            }
            self.reset = false;
            self.animation.next();
        }

        self.time += delta;

        match self.robot_state {
            RobotState::Homing => self.home_process(owner),
            RobotState::Following => self.follow_process(owner),
        }
    }

    fn set_body_height(&mut self, owner: &Spatial, height: f64) {
        self.body_height = height;
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
        let node: Ref<Node> = unsafe { owner.upcast::<Node>().assume_shared() };
        let safe_node = unsafe { node.assume_safe() };
        let safe_node = safe_node.get_node("Body").unwrap();
        let body = body_from_node(&safe_node);
        body.set_radius(radius);
    }

    fn home_process(&mut self, owner: &Spatial) {
        if self.time > 2.0 {
            self.time = 0.0;
            self.robot_state = RobotState::Following;
            return;
        }

        for leg in &mut self.legs {
            unsafe {
                Self::move_leg(owner, leg, Point3::new(0.0, 90.0, 190.0));
            }
        }
    }

    unsafe fn follow_process(&mut self, owner: &Spatial) {
        if self.time >= self.step_delay {
            self.time = 0.0;
            let (reset, animation_duration) = self.animation.next();
            self.reset = !reset;
            if !self.reset {
                for node in self.tracking_nodes.drain(..) {
                    owner.remove_child(node);
                }
            }
            self.step_delay = animation_duration * self.anim_scale;
        }

        // self.set_body_height(
        //     owner,
        //     self.base_height
        //         + (HEIGHT_MOVEMENT
        //             * (self.time * 4.0 * std::f64::consts::PI / self.step_delay).sin()
        //             + HEIGHT_MOVEMENT),
        // );

        // Figure out how far into the animation we need to render.
        let interpolation = (self.time / self.step_delay) * std::f64::consts::TAU;
        // Get the targets for each foot.
        let targets = self
            .animation
            .targets(self.walking_radius as f32, interpolation);

        // Calculate the servo angles for each leg.
        for (target, leg) in targets.into_iter().zip(self.legs.iter_mut()) {
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
                owner.add_child(node, false);
            }

            // Get target in each leg's coordinate space.
            let target = leg
                .origin_to_coxa(self.body_height as f32, self.base_radius as f32)
                .transform_point(&target);

            Self::move_leg(owner, leg, target);
        }
    }

    unsafe fn move_leg(owner: &Spatial, mecha_leg: &mut MechaLeg, target: Point3<f32>) {
        let node_name = mecha_leg.servo_name();
        let leg = &mut mecha_leg.leg;
        if let Err(e) = leg.go_to(target) {
            godot_print!(
                "Failed to calculate leg position for leg {}: {e:?}",
                mecha_leg.idx
            );
            return;
        }

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
