use crate::{consts::*, log};
use communication::{I2cRequest, I2cRequestField, I2cRequestOp};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use nalgebra::{Translation3, UnitQuaternion, Vector3};
use scapegoat::SgSet;
use state::{RobotState, StateMachine};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum CommandErr {
    InvalidFloat,
}

#[derive(Copy, Clone)]
pub(crate) enum SyncKind {
    OnlyCommands,
    AllState,
}

#[derive(Copy, Clone)]
pub(crate) enum Command {
    Sync(SyncKind),
    ChangeState(StateMachine),
    SetAnimationFactor(f32),
    SetBodyTranslation(Translation3<f32>),
    SetBodyRotation(UnitQuaternion<f32>),
    SetMotionVector(Vector3<f32>),
    SetAngularVelocity(f32),
    SetLegRadius(f32),
    SetBatteryUpdateInterval(u32),
}

#[cfg(feature = "defmt")]
impl defmt::Format for Command {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Command::Sync(kind) => match kind {
                SyncKind::AllState => defmt::write!(f, "Sync(AllState)"),
                SyncKind::OnlyCommands => defmt::write!(f, "Sync(OnlyCommands)"),
            },
            Command::ChangeState(sm) => match sm {
                StateMachine::Paused => defmt::write!(f, "ChangeState(Paused)"),
                StateMachine::Homing => defmt::write!(f, "ChangeState(Homing)"),
                StateMachine::Calibrating => defmt::write!(f, "ChangeState(Calibrating)"),
                StateMachine::Looping => defmt::write!(f, "ChangeState(Looping)"),
                StateMachine::Exploring => defmt::write!(f, "ChangeState(Exploring)"),
            },
            Command::SetAnimationFactor(s) => defmt::write!(f, "SetAnimationFactor({})", s),
            Command::SetBodyTranslation(t) => {
                defmt::write!(f, "SetBodyTranslation({}, {}, {})", t.x, t.y, t.z)
            }
            Command::SetBodyRotation(r) => {
                defmt::write!(f, "SetBodyRotation({},{},{},{})", r.i, r.j, r.k, r.w)
            }
            Command::SetMotionVector(v) => {
                defmt::write!(f, "SetMotionVector({}, {}, {})", v.x, v.y, v.z)
            }
            Command::SetAngularVelocity(v) => defmt::write!(f, "SetAngularVelocity({})", v),
            Command::SetLegRadius(r) => defmt::write!(f, "SetLegRadius({})", r),
            Command::SetBatteryUpdateInterval(i) => {
                defmt::write!(f, "SetBatteryUpdateInterval({})", i)
            }
        }
    }
}

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum UpdateKind {
    Notify,
    Indicate,
}

impl UpdateKind {
    pub(crate) fn from_cccd(notification: bool, indicate: bool) -> Option<UpdateKind> {
        match (notification, indicate) {
            (true, _) => Some(UpdateKind::Notify),
            (_, true) => Some(UpdateKind::Indicate),
            _ => None,
        }
    }
}

pub(crate) struct CommandUpdate {
    pub(crate) state: Option<UpdateKind>,
    pub(crate) animation_factor: Option<UpdateKind>,
    pub(crate) body_translation: Option<UpdateKind>,
    pub(crate) body_rotation: Option<UpdateKind>,
    pub(crate) motion_vector: Option<UpdateKind>,
    pub(crate) angular_velocity: Option<UpdateKind>,
    pub(crate) leg_radius: Option<UpdateKind>,
    pub(crate) battery_interval: Option<UpdateKind>,
}

impl CommandUpdate {
    pub(crate) fn new() -> Self {
        Self {
            state: None,
            animation_factor: None,
            body_translation: None,
            body_rotation: None,
            motion_vector: None,
            angular_velocity: None,
            leg_radius: None,
            battery_interval: None,
        }
    }

    pub(crate) fn get(&self, command: Command) -> Option<UpdateKind> {
        match command {
            Command::Sync(_) => None,
            Command::ChangeState(_) => self.state,
            Command::SetAnimationFactor(_) => self.animation_factor,
            Command::SetBodyTranslation(_) => self.body_translation,
            Command::SetBodyRotation(_) => self.body_rotation,
            Command::SetMotionVector(_) => self.motion_vector,
            Command::SetAngularVelocity(_) => self.angular_velocity,
            Command::SetLegRadius(_) => self.leg_radius,
            Command::SetBatteryUpdateInterval(_) => self.battery_interval,
        }
    }
}

pub(crate) struct StateManager {
    should_update: SgSet<I2cRequestOp, 16>,
}

impl StateManager {
    pub(crate) fn new() -> Self {
        Self {
            should_update: SgSet::new(),
        }
    }
}

pub(crate) async fn handle_command(
    robot_state: &mut RobotState,
    state_manager: &mut StateManager,
    sender: Sender<'static, CriticalSectionRawMutex, I2cRequest, 1>,
    command: Command,
) -> Result<(), CommandErr> {
    match command {
        Command::Sync(kind) => {
            for op in state_manager.should_update.iter() {
                log::info!("Sending update {:?}", op);
                send_op(robot_state, &sender, *op).await;
            }
            if let SyncKind::AllState = kind {
                send_op(
                    robot_state,
                    &sender,
                    I2cRequestOp::Get(I2cRequestField::AngularVelocity),
                )
                .await;
                send_op(
                    robot_state,
                    &sender,
                    I2cRequestOp::Get(I2cRequestField::BodyRotation),
                )
                .await;
                send_op(
                    robot_state,
                    &sender,
                    I2cRequestOp::Get(I2cRequestField::BodyTranslation),
                )
                .await;
                send_op(
                    robot_state,
                    &sender,
                    I2cRequestOp::Get(I2cRequestField::LegRadius),
                )
                .await;
                send_op(
                    robot_state,
                    &sender,
                    I2cRequestOp::Get(I2cRequestField::MotionVector),
                )
                .await;
                send_op(
                    robot_state,
                    &sender,
                    I2cRequestOp::Get(I2cRequestField::AnimationFactor),
                )
                .await;
            }

            state_manager.should_update.clear();
        }
        Command::ChangeState(sm) => {
            if robot_state.state_machine != sm {
                robot_state.state_machine = sm;
                let _ = state_manager
                    .should_update
                    .insert(I2cRequestOp::ChangeState);
            }
        }
        Command::SetAnimationFactor(val) => {
            if robot_state.animation_factor != val {
                if (MIN_SPEED..=MAX_SPEED).contains(&val) {
                    robot_state.animation_factor = val;
                    let _ = state_manager
                        .should_update
                        .insert(I2cRequestOp::Set(I2cRequestField::AnimationFactor));
                } else {
                    return Err(CommandErr::InvalidFloat);
                }
            }
        }
        Command::SetBodyTranslation(val) => {
            if robot_state.body_translation != val {
                if (MIN_X..=MAX_X).contains(&val.x)
                    && (MIN_Y..=MAX_Y).contains(&val.y)
                    && (MIN_Z..=MAX_Z).contains(&val.z)
                {
                    robot_state.body_translation = val;
                    let _ = state_manager
                        .should_update
                        .insert(I2cRequestOp::Set(I2cRequestField::BodyTranslation));
                } else {
                    return Err(CommandErr::InvalidFloat);
                }
            }
        }
        Command::SetBodyRotation(val) => {
            if robot_state.body_rotation != val {
                let (roll, pitch, yaw) = val.euler_angles();
                if (MIN_ROLL..=MAX_ROLL).contains(&roll)
                    && (MIN_PITCH..=MAX_PITCH).contains(&pitch)
                    && (MIN_YAW..=MAX_YAW).contains(&yaw)
                {
                    robot_state.body_rotation = val;
                    let _ = state_manager
                        .should_update
                        .insert(I2cRequestOp::Set(I2cRequestField::BodyRotation));
                } else {
                    return Err(CommandErr::InvalidFloat);
                }
            }
        }
        Command::SetMotionVector(val) => {
            if robot_state.motion_vector != val {
                if (MIN_X..=MAX_X).contains(&val.x)
                    && (MIN_Y..=MAX_Y).contains(&val.y)
                    && (MIN_Z..=MAX_Z).contains(&val.z)
                {
                    robot_state.motion_vector = val;
                    let _ = state_manager
                        .should_update
                        .insert(I2cRequestOp::Set(I2cRequestField::MotionVector));
                } else {
                    return Err(CommandErr::InvalidFloat);
                }
            }
        }
        Command::SetAngularVelocity(val) => {
            if robot_state.angular_velocity != val {
                if (MIN_AVEL..=MAX_AVEL).contains(&val) {
                    robot_state.angular_velocity = val;
                    let _ = state_manager
                        .should_update
                        .insert(I2cRequestOp::Set(I2cRequestField::AngularVelocity));
                } else {
                    return Err(CommandErr::InvalidFloat);
                }
            }
        }
        Command::SetLegRadius(val) => {
            if robot_state.leg_radius != val {
                if (MIN_LRAD..=MAX_LRAD).contains(&val) {
                    robot_state.leg_radius = val;
                    let _ = state_manager
                        .should_update
                        .insert(I2cRequestOp::Set(I2cRequestField::LegRadius));
                } else {
                    return Err(CommandErr::InvalidFloat);
                }
            }
        }
        Command::SetBatteryUpdateInterval(val) => {
            if robot_state.battery_update_interval_ms != val {
                if (MIN_UPIVAL..=MAX_UPIVAL).contains(&val) {
                    robot_state.battery_update_interval_ms = val;
                    let _ = state_manager
                        .should_update
                        .insert(I2cRequestOp::SetBatteryUpdateInterval);
                } else {
                    return Err(CommandErr::InvalidFloat);
                }
            }
        }
    }

    Ok(())
}

async fn send_op(
    robot_state: &mut RobotState,
    sender: &Sender<'static, CriticalSectionRawMutex, I2cRequest, 1>,
    op: I2cRequestOp,
) {
    let i2c_message = match op {
        I2cRequestOp::ChangeState => I2cRequest::ChangeState(robot_state.state_machine),
        I2cRequestOp::Set(field) => match field {
            I2cRequestField::AnimationFactor => {
                I2cRequest::SetAnimationFactor(robot_state.animation_factor)
            }
            I2cRequestField::AngularVelocity => {
                I2cRequest::SetAngularVelocity(robot_state.angular_velocity)
            }
            I2cRequestField::MotionVector => I2cRequest::SetMotionVector(
                robot_state.motion_vector.x,
                robot_state.motion_vector.y,
                robot_state.motion_vector.z,
            ),
            I2cRequestField::BodyTranslation => {
                log::info!(
                    "Sending body translation (x:{}, y:{}, z:{})",
                    robot_state.body_translation.x,
                    robot_state.body_translation.y,
                    robot_state.body_translation.z,
                );
                I2cRequest::SetBodyTranslation(
                    robot_state.body_translation.x,
                    robot_state.body_translation.y,
                    robot_state.body_translation.z,
                )
            }
            I2cRequestField::BodyRotation => {
                let (r, p, y) = robot_state.body_rotation.euler_angles();
                I2cRequest::SetBodyRotation(r, p, y)
            }
            I2cRequestField::LegRadius => I2cRequest::SetLegRadius(robot_state.leg_radius),
        },
        I2cRequestOp::SetBatteryUpdateInterval => {
            I2cRequest::SetBatteryUpdateInterval(robot_state.battery_update_interval_ms)
        }
        I2cRequestOp::Get(field) => match field {
            I2cRequestField::AnimationFactor => I2cRequest::GetAnimationFactor,
            I2cRequestField::AngularVelocity => I2cRequest::GetAngularVelocity,
            I2cRequestField::MotionVector => I2cRequest::GetMotionVector,
            I2cRequestField::BodyTranslation => I2cRequest::GetBodyTranslation,
            I2cRequestField::BodyRotation => I2cRequest::GetBodyRotation,
            I2cRequestField::LegRadius => I2cRequest::GetLegRadius,
        },
        I2cRequestOp::GetBatteryLevel => I2cRequest::GetBatteryLevel,
    };
    sender.send(i2c_message).await;
}
