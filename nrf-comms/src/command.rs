use crate::{consts::*, log, RobotState};
use bluetooth_comms::{CalibrationDatum, CalibrationIndex, SetRes, SetResult};
use communication::{I2cRequest, I2cRequestField, I2cRequestOp};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use nalgebra::{Translation3, UnitQuaternion, Vector3};
use scapegoat::SgSet;
use state::StateMachine;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum CommandErr {
    InvalidFloat,
    InvalidIndex,
}

#[derive(Copy, Clone)]
pub(crate) enum SyncKind {
    OnlyCommands,
    AllState,
}

#[cfg(feature = "defmt")]
impl defmt::Format for Command {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "CalibrationIndex {{ leg: {}, joint: {}, kind: {} }}",
            self.leg,
            self.joint,
            self.kind
        );
    }
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
    GetCalibrationFor(CalibrationIndex),
    SetCalibrationDatum(f32, CalibrationIndex),
}

#[derive(Copy, Clone)]
pub(crate) enum CommandResult {
    Sync(u8),
    ChangeState(StateMachine),
    SetAnimationFactor(f32),
    SetBodyTranslation(Translation3<f32>),
    SetBodyRotation(UnitQuaternion<f32>),
    SetMotionVector(Vector3<f32>),
    SetAngularVelocity(f32),
    SetLegRadius(f32),
    SetBatteryUpdateInterval(u32),
    GetCalibrationFor(CalibrationDatum),
    SetCalibrationDatum(SetResult),
}

impl Command {
    pub(crate) fn to_result(
        self,
        robot_state: &RobotState,
        result: Result<(), CommandErr>,
    ) -> CommandResult {
        match (self, result) {
            (Command::Sync(_), _) => CommandResult::Sync(robot_state.battery_level as u8),
            (Command::ChangeState(_), _) => CommandResult::ChangeState(robot_state.state_machine),
            (Command::SetAnimationFactor(_), _) => {
                CommandResult::SetAnimationFactor(robot_state.animation_factor)
            }
            (Command::SetBodyTranslation(_), _) => {
                CommandResult::SetBodyTranslation(robot_state.body_translation)
            }
            (Command::SetBodyRotation(_), _) => {
                CommandResult::SetBodyRotation(robot_state.body_rotation)
            }
            (Command::SetMotionVector(_), _) => {
                CommandResult::SetMotionVector(robot_state.motion_vector)
            }
            (Command::SetAngularVelocity(_), _) => {
                CommandResult::SetAngularVelocity(robot_state.angular_velocity)
            }
            (Command::SetLegRadius(_), _) => CommandResult::SetLegRadius(robot_state.leg_radius),
            (Command::SetBatteryUpdateInterval(_), _) => {
                CommandResult::SetBatteryUpdateInterval(robot_state.battery_update_interval_ms)
            }
            (Command::GetCalibrationFor(index), Ok(())) => {
                let cal = robot_state
                    .joint(index.leg as usize, index.joint as usize)
                    .and_then(|joint| joint.cal.get(index.kind as usize))
                    .map(|cal| cal.pulse_ms)
                    // Unwrap ok because we're in the Ok arm.
                    .unwrap();
                CommandResult::GetCalibrationFor(CalibrationDatum::new(cal, index))
            }
            (Command::GetCalibrationFor(index), Err(e)) => {
                log::error!("Error getting calibration for {:?}: {:?}", index, e);
                let cal = robot_state
                    .joint(index.leg as usize, index.joint as usize)
                    .and_then(|joint| joint.cal.get(index.kind as usize))
                    .map(|cal| cal.pulse_ms)
                    // Unwrap ok because we're in the Ok arm.
                    .unwrap();
                CommandResult::GetCalibrationFor(CalibrationDatum::new(cal, index))
            }
            (Command::SetCalibrationDatum(_, index), Ok(())) => {
                CommandResult::SetCalibrationDatum(SetResult {
                    index,
                    result: SetRes::Ok,
                })
            }
            (Command::SetCalibrationDatum(_, index), Err(e)) => {
                CommandResult::SetCalibrationDatum(SetResult {
                    index,
                    result: match e {
                        CommandErr::InvalidFloat => SetRes::InvalidValue,
                        CommandErr::InvalidIndex => SetRes::InvalidIndex,
                    },
                })
            }
        }
    }
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
            Command::GetCalibrationFor(index) => defmt::write!(f, "GetCalibrationFor({})", index),
            Command::SetCalibrationDatum(pulse, index) => {
                defmt::write!(f, "SetCalibrationDatum({}, {})", pulse, index)
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
    pub(crate) get_calibration_result: Option<UpdateKind>,
    pub(crate) set_calibration_result: Option<UpdateKind>,
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
            get_calibration_result: None,
            set_calibration_result: None,
        }
    }

    pub(crate) fn get(&self, command: CommandResult) -> Option<UpdateKind> {
        match command {
            CommandResult::Sync(_) => None,
            CommandResult::ChangeState(_) => self.state,
            CommandResult::SetAnimationFactor(_) => self.animation_factor,
            CommandResult::SetBodyTranslation(_) => self.body_translation,
            CommandResult::SetBodyRotation(_) => self.body_rotation,
            CommandResult::SetMotionVector(_) => self.motion_vector,
            CommandResult::SetAngularVelocity(_) => self.angular_velocity,
            CommandResult::SetLegRadius(_) => self.leg_radius,
            CommandResult::SetBatteryUpdateInterval(_) => self.battery_interval,
            CommandResult::GetCalibrationFor(_) => self.get_calibration_result,
            CommandResult::SetCalibrationDatum(_) => self.set_calibration_result,
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
        Command::GetCalibrationFor(index) => {
            if robot_state
                .joint(index.leg as usize, index.joint as usize)
                .and_then(|j| j.cal.get(index.kind as usize))
                .is_none()
            {
                return Err(CommandErr::InvalidIndex);
            }
            let _ = state_manager.should_update.insert(I2cRequestOp::Get(
                I2cRequestField::Calibration {
                    leg: index.leg,
                    joint: index.joint,
                    kind: index.kind,
                },
            ));
        }
        Command::SetCalibrationDatum(pulse, index) => {
            let Some(cal) = robot_state
                .joint_mut(index.leg as usize, index.joint as usize)
                .and_then(|j| j.cal.get_mut(index.kind as usize))
            else {
                return Err(CommandErr::InvalidIndex);
            };
            cal.pulse_ms = pulse;
            let _ = state_manager.should_update.insert(I2cRequestOp::Set(
                I2cRequestField::Calibration {
                    leg: index.leg,
                    joint: index.joint,
                    kind: index.kind,
                },
            ));
        }
    }

    Ok(())
}

async fn send_op(
    robot_state: &RobotState,
    sender: &Sender<'static, CriticalSectionRawMutex, I2cRequest, 1>,
    op: I2cRequestOp,
) {
    let i2c_message = match op {
        I2cRequestOp::ChangeState => Some(I2cRequest::ChangeState(robot_state.state_machine)),
        I2cRequestOp::Set(field) => match field {
            I2cRequestField::AnimationFactor => {
                Some(I2cRequest::SetAnimationFactor(robot_state.animation_factor))
            }
            I2cRequestField::AngularVelocity => {
                Some(I2cRequest::SetAngularVelocity(robot_state.angular_velocity))
            }
            I2cRequestField::MotionVector => Some(I2cRequest::SetMotionVector(
                robot_state.motion_vector.x,
                robot_state.motion_vector.y,
                robot_state.motion_vector.z,
            )),
            I2cRequestField::BodyTranslation => Some(I2cRequest::SetBodyTranslation(
                robot_state.body_translation.x,
                robot_state.body_translation.y,
                robot_state.body_translation.z,
            )),
            I2cRequestField::BodyRotation => {
                let (r, p, y) = robot_state.body_rotation.euler_angles();
                Some(I2cRequest::SetBodyRotation(r, p, y))
            }
            I2cRequestField::LegRadius => Some(I2cRequest::SetLegRadius(robot_state.leg_radius)),
            I2cRequestField::Calibration { leg, joint, kind } => robot_state
                .joints
                .and_then(|j| j.get(leg as usize).copied())
                .and_then(|l| l.get(joint as usize).copied())
                .and_then(|j| j.cal.get(kind as usize).copied())
                .map(|c| I2cRequest::SetCalibration(leg, joint, kind, c.pulse_ms)),
        },
        I2cRequestOp::SetBatteryUpdateInterval => Some(I2cRequest::SetBatteryUpdateInterval(
            robot_state.battery_update_interval_ms,
        )),
        I2cRequestOp::Get(field) => Some(match field {
            I2cRequestField::AnimationFactor => I2cRequest::GetAnimationFactor,
            I2cRequestField::AngularVelocity => I2cRequest::GetAngularVelocity,
            I2cRequestField::MotionVector => I2cRequest::GetMotionVector,
            I2cRequestField::BodyTranslation => I2cRequest::GetBodyTranslation,
            I2cRequestField::BodyRotation => I2cRequest::GetBodyRotation,
            I2cRequestField::LegRadius => I2cRequest::GetLegRadius,
            I2cRequestField::Calibration { leg, joint, kind } => {
                I2cRequest::GetCalibration { leg, joint, kind }
            }
        }),
        I2cRequestOp::GetBatteryLevel => Some(I2cRequest::GetBatteryLevel),
    };
    if let Some(i2c_message) = i2c_message {
        sender.send(i2c_message).await;
    }
}
