use crate::servo::Servo;
use cgmath::Deg;
use cgmath::Point3;
use cgmath::Rad;
use gdnative::api::*;
use gdnative::prelude::*;
use kinematics::Leg;

/// The Game "class"
#[derive(NativeClass)]
#[inherit(Spatial)]
#[register_with(Self::register_builder)]
pub struct Game {
    name: String,
    time: f64,
    #[property]
    step_delay: f64,
    leg: Leg,
    reset: bool,
}

// __One__ `impl` block can have the `#[methods]` attribute, which will generate
// code to automatically bind any exported methods to Godot.
#[methods]
impl Game {
    // Register the builder for methods, properties and/or signals.
    fn register_builder(_builder: &ClassBuilder<Self>) {
        godot_print!("Game builder is registered!");
    }

    /// The "constructor" of the class.
    fn new(_owner: &Spatial) -> Self {
        godot_print!("Game is created!");
        Game {
            name: "".to_string(),
            time: 0.0,
            step_delay: 0.0,
            leg: Leg::new(),
            reset: false,
        }
    }

    // In order to make a method known to Godot, the #[export] attribute has to be used.
    // In Godot script-classes do not actually inherit the parent class.
    // Instead they are "attached" to the parent object, called the "owner".
    // The owner is passed to every single exposed method.
    #[method]
    unsafe fn _ready(&mut self, #[base] _owner: &Spatial) {
        // The `godot_print!` macro works like `println!` but prints to the Godot-editor
        // output tab as well.
        self.name = "Game".to_string();
        godot_print!("{} is ready!", self.name);
    }

    // This function will be called in every frame
    #[method]
    unsafe fn _process(&mut self, #[base] owner: &Spatial, delta: f64) {
        self.time += delta;
        if self.time >= self.step_delay {
            self.reset = true;
            self.time = 0.0;
        }

        let circle_angle = self.time / self.step_delay * 2.0 * std::f64::consts::PI;
        let target = point_on_circle(0.0, 200.0, 25.0, circle_angle as f32);
        let point = Point3 {
            x: target.x,
            y: target.y,
            z: target.z,
        };
        if !self.reset {
            let node = CSGSphere::new();
            node.translate(Vector3 {
                x: -point.x,
                y: point.z,
                z: point.y,
            });
            node.set_radius(0.5);
            owner.add_child(node, false);
        }
        godot_print!(
            "targetting ({}, {}, {}) matching angle {}",
            point.x,
            point.y,
            point.z,
            Deg::from(Rad(circle_angle)).0
        );
        if let Err(e) = self.leg.go_to(point) {
            godot_print!("Failed to calculate leg position: {e:?}");
            return;
        }

        let mut node: Ref<Node> = owner.upcast::<Node>().assume_shared();
        let safe_node = node.assume_safe();
        let safe_node = safe_node.get_node("CoxaServo").unwrap();
        let servo = servo_from_node(&safe_node);
        let mut next_node = servo
            .map_mut(|servo, _cylinder| {
                servo.set_target_angle(self.leg.coxa_servo_angle);
                servo.next_servo()
            })
            .unwrap();

        node = servo.base().upcast::<Node>().assume_shared();
        let safe_node = node.assume_safe();
        let safe_node = safe_node.get_node(next_node.unwrap()).unwrap();
        let servo = servo_from_node(&safe_node);
        next_node = servo
            .map_mut(|servo, _cylinder| {
                servo.set_target_angle(self.leg.femur_servo_angle);
                servo.next_servo()
            })
            .unwrap();

        node = servo.base().upcast::<Node>().assume_shared();
        let safe_node = node.assume_safe();
        let safe_node = safe_node.get_node(next_node.unwrap()).unwrap();
        let servo = servo_from_node(&safe_node);
        servo
            .map_mut(|servo, _cylinder| {
                servo.set_target_angle(self.leg.tibia_servo_angle);
                servo.next_servo()
            })
            .unwrap();
    }
}

fn servo_from_node(node: &Ref<Node>) -> TInstance<'_, Servo> {
    unsafe {
        let node = node.assume_safe();
        let node = node.cast::<CSGCylinder>().unwrap();
        node.cast_instance::<Servo>().unwrap()
    }
}

fn point_on_circle(x: f32, y: f32, radius: f32, angle: f32) -> Point3<f32> {
    Point3::new(x + radius * angle.cos(), y + radius * angle.sin(), 0.0)
}
