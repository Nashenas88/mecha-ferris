use crate::servo::Servo;
use gdnative::api::*;
use gdnative::prelude::*;
use kinematics::{DefaultConsts, ExpensiveMath, Leg, LegConsts, LutMath, Point3, Translation3};

type MechaLeg = Leg<f32, StdMath>;

#[derive(Copy, Clone)]
pub struct StdMath;

impl ExpensiveMath<f32> for StdMath {
    fn atan2(l: f32, r: f32) -> f32 {
        l.atan2(r)
    }

    #[inline(always)]
    fn acos(f: f32) -> f32 {
        f.acos()
    }

    #[inline(always)]
    fn sin(f: f32) -> f32 {
        f.sin()
    }

    #[inline(always)]
    fn cos(f: f32) -> f32 {
        f.cos()
    }

    #[inline(always)]
    fn sincos(f: f32) -> (f32, f32) {
        (f.sin(), f.cos())
    }

    #[inline(always)]
    fn sqrt(f: f32) -> f32 {
        f.sqrt()
    }
}

/// The Game "class"
#[derive(NativeClass)]
#[inherit(Spatial)]
#[register_with(Self::register_builder)]
pub struct Game {
    time: f64,
    #[property]
    step_delay: f64,
    leg: MechaLeg,
    reset: bool,
    #[property]
    target: NodePath,
    #[property]
    target_coxa_space: NodePath,
    #[property]
    target_femur_space: NodePath,
    active_space: Space,
    robot_state: RobotState,
}

#[derive(Copy, Clone, Debug)]
enum Space {
    FixedFrame,
    CoxaServo,
    FemurServo,
}

enum RobotState {
    Homing,
    Following,
}

impl Space {
    fn next(&mut self) {
        *self = match self {
            Space::FixedFrame => Space::CoxaServo,
            Space::CoxaServo => Space::FemurServo,
            Space::FemurServo => Space::FixedFrame,
        }
    }

    fn frame_node_path(&self) -> NodePath {
        match self {
            Space::FixedFrame => "CoxaServo".into(),
            Space::CoxaServo => "CoxaServoCoxaSpace".into(),
            Space::FemurServo => "FemurServoFemurSpace".into(),
        }
    }
}

#[methods]
impl Game {
    // Register the builder for methods, properties and/or signals.
    fn register_builder(_builder: &ClassBuilder<Self>) {
        godot_print!("Game builder is registered!");
    }

    /// The "constructor" of the class.
    fn new(_owner: &Spatial) -> Self {
        Game {
            time: 0.0,
            step_delay: 0.0,
            leg: MechaLeg::default(),
            reset: false,
            target: NodePath::default(),
            target_coxa_space: NodePath::default(),
            target_femur_space: NodePath::default(),
            active_space: Space::FixedFrame,
            robot_state: RobotState::Homing,
        }
    }

    #[method]
    unsafe fn _ready(&mut self, #[base] _owner: &Spatial) {}

    // This function will be called in every frame
    #[method]
    unsafe fn _process(&mut self, #[base] owner: &Spatial, delta: f64) {
        let input = Input::godot_singleton();
        if input.is_action_just_pressed("change_space", true) {
            let old_space = self.active_space;
            self.active_space.next();
            let node = owner
                .get_node(old_space.frame_node_path())
                .unwrap()
                .assume_safe();
            let node = node.cast::<Spatial>().unwrap();
            node.set_visible(false);
            let node = owner
                .get_node(self.target_node_for_space(old_space))
                .unwrap()
                .assume_safe();
            let node = node.cast::<Spatial>().unwrap();
            node.set_visible(false);

            let node = owner
                .get_node(self.active_space.frame_node_path())
                .unwrap()
                .assume_safe();
            let node = node.cast::<Spatial>().unwrap();
            node.set_visible(true);
            let node = owner
                .get_node(self.target_node_for_space(self.active_space))
                .unwrap()
                .assume_safe();
            let node = node.cast::<Spatial>().unwrap();
            node.set_visible(true);
        }

        self.time += delta;

        match self.robot_state {
            RobotState::Homing => self.home_process(owner),
            RobotState::Following => self.follow_process(owner),
        }
    }

    fn home_process(&mut self, owner: &Spatial) {
        if self.time > 2.0 {
            self.time = 0.0;
            self.robot_state = RobotState::Following;
            return;
        }

        unsafe {
            self.move_leg(owner, Point3::new(0.0, 90.0, 190.0));
        }
    }

    unsafe fn follow_process(&mut self, owner: &Spatial) {
        if self.time >= self.step_delay {
            self.reset = true;
            self.time = 0.0;
        }
        let circle_angle = self.time / self.step_delay * 2.0 * std::f64::consts::PI;
        let target = point_on_circle(0.0, -60.0, 180.0, 40.0, circle_angle as f32);
        // let target = point_on_line(0.0, -70.0, 140.0, 40.0, circle_angle as f32);
        let vector = Vector3 {
            x: target.x,
            y: target.y,
            z: target.z,
        };
        if !self.reset {
            let node = CSGSphere::new();
            node.translate(vector);
            node.set_radius(0.5);
            owner.add_child(node, false);
        }

        if !self.target.to_string().is_empty() {
            let node = owner.get_node(self.target.to_string()).unwrap();
            let node = node.assume_safe();
            let node = node.cast::<CSGSphere>().unwrap();
            node.set_translation(vector);
        }

        self.move_leg(owner, target);
    }

    unsafe fn move_leg(&mut self, owner: &Spatial, target: Point3<f32>) {
        // Move the target to the coxa servo's space.
        let target = Translation3::new(0.0, -<DefaultConsts as LegConsts>::COXA_HEIGHT, 0.0)
            .transform_point(&target);
        godot_print!("Target at ({}, {}, {})", target.x, target.y, target.z);
        if let Err(e) = self.leg.go_to(target) {
            godot_print!("Failed to calculate leg position: {e:?}");
            return;
        }

        godot_print!(
            "Angles: ({:6.2}, {:6.2}, {:6.2})",
            rad_to_degree(self.leg.coxa_servo_angle()),
            rad_to_degree(self.leg.femur_servo_angle()),
            rad_to_degree(self.leg.tibia_servo_angle())
        );

        let origin_to_coxa = Leg::<f32, LutMath, DefaultConsts>::origin_to_coxa();
        let target_coxa_space = self.target_coxa_space.to_string();
        if !target_coxa_space.is_empty() {
            let node = owner.get_node(target_coxa_space).unwrap().assume_safe();
            let node = node.cast::<CSGSphere>().unwrap();
            let target = origin_to_coxa.transform_point(&target);
            node.set_translation(Vector3::new(target.x, target.y, target.z));
        }

        let coxa_to_femur =
            Leg::<f32, LutMath, DefaultConsts>::coxa_to_femur(self.leg.coxa_servo_angle());
        let target_femur_space = self.target_femur_space.to_string();
        if !target_femur_space.is_empty() {
            let node = owner.get_node(target_femur_space).unwrap().assume_safe();
            let node = node.cast::<CSGSphere>().unwrap();
            let target = (coxa_to_femur * origin_to_coxa).transform_point(&target);
            node.set_translation(Vector3::new(target.x, target.y, target.z));
        }

        let deg_coxa_servo_angle = rad_to_degree(self.leg.coxa_servo_angle());
        let deg_femur_servo_angle = rad_to_degree(self.leg.femur_servo_angle());
        let deg_tibia_servo_angle = rad_to_degree(self.leg.tibia_servo_angle());

        let mut node: Ref<Node> = owner.upcast::<Node>().assume_shared();
        let safe_node = node.assume_safe();
        let safe_node = safe_node.get_node("CoxaServo").unwrap();
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

        let mut node: Ref<Node> = owner.upcast::<Node>().assume_shared();
        let safe_node = node.assume_safe();
        let safe_node = safe_node.get_node("CoxaServoCoxaSpace").unwrap();
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

        let mut node: Ref<Node> = owner.upcast::<Node>().assume_shared();
        let safe_node = node.assume_safe();
        let safe_node = safe_node.get_node("FemurServoFemurSpace").unwrap();
        let servo = servo_from_node(&safe_node);
        let next_node = servo
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

    fn target_node_for_space(&self, space: Space) -> GodotString {
        match space {
            Space::FixedFrame => self.target.to_godot_string(),
            Space::CoxaServo => self.target_coxa_space.to_godot_string(),
            Space::FemurServo => self.target_femur_space.to_godot_string(),
        }
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

#[allow(dead_code)]
fn point_on_circle(z: f32, y: f32, x: f32, radius: f32, angle: f32) -> Point3<f32> {
    Point3::new(x + radius * angle.sin(), y, z + radius * angle.cos())
}

#[allow(dead_code)]
fn point_on_line(z: f32, y: f32, x: f32, length: f32, t: f32) -> Point3<f32> {
    Point3::new(x, y, z + length * t.cos())
}
