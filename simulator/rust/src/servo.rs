use gdnative::api::*;
use gdnative::prelude::*;

#[derive(NativeClass)]
#[inherit(CSGCylinder)]
#[register_with(Self::register_builder)]
pub struct Servo {
    angle: f32,
    target_angle: f32,
    #[property(path = "base/min_angle")]
    min_angle: f32,
    #[property(path = "base/max_angle")]
    max_angle: f32,
    #[property(path = "base/next_servo")]
    next_servo: Option<NodePath>,
    #[property(path = "base/angular_velocity")]
    angular_velocity: f32,
    time: f32,
}

#[methods]
impl Servo {
    fn register_builder(_builder: &ClassBuilder<Self>) {
        godot_print!("Leg builder is registered!");
    }

    /// The "constructor" of the class.
    fn new(_owner: &CSGCylinder) -> Self {
        Self {
            angle: 0.0,
            target_angle: 0.0,
            min_angle: 0.0,
            max_angle: 0.0,
            next_servo: None,
            angular_velocity: 0.0,
            time: 0.0,
        }
    }

    #[method]
    unsafe fn _ready(&mut self, #[base] owner: &CSGCylinder) {
        owner.set_physics_process(true);
    }

    #[method]
    unsafe fn _physics_process(&mut self, #[base] owner: &CSGCylinder, delta: f64) {
        self.time += delta as f32;

        if self.target_angle != self.angle {
            let angle_error = self.target_angle - self.angle;
            self.angle += angle_error.signum() * self.angular_velocity * delta as f32;
            if (self.target_angle - self.angle).signum() != angle_error.signum() {
                self.angle = self.target_angle;
            }
            owner.set_rotation_degrees(Vector3::new(0.0, self.angle, 0.0));
        }
    }

    pub fn set_target_angle(&mut self, angle: f32) {
        self.target_angle = angle;
    }

    pub fn next_servo(&self) -> Option<String> {
        self.next_servo.as_ref().map(|np| np.to_string())
    }
}
