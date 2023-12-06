use crate::{consts::*, log, CommsError, CriticalReceiver, CriticalSender, RobotState};
use bluetooth_comms::{CalibrationDatum, CalibrationIndex, SetRes, SetResult};
use communication::{I2cRequest, I2cResponse};
use heapless::Vec;
use nalgebra::{Translation3, UnitQuaternion, Vector3};
use state::StateMachine;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum CommandErr {
    InvalidFloat,
    InvalidIndex,
}

#[derive(Copy, Clone)]
pub(crate) enum Command {
    /// Send all local state to the client.
    NotifyAll,
    /// Send all state updates to the robot.
    Sync,
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

impl From<I2cResponse> for CommandResult {
    fn from(value: I2cResponse) -> Self {
        match value {
            I2cResponse::GetState(sm) => CommandResult::GetState(sm),
            I2cResponse::GetAnimationFactor(f) => CommandResult::GetAnimationFactor(f),
            I2cResponse::GetAngularVelocity(f) => CommandResult::GetAngularVelocity(f),
            I2cResponse::GetMotionVector(v) => CommandResult::GetMotionVector(v),
            I2cResponse::GetBodyTranslation(t) => CommandResult::GetBodyTranslation(t),
            I2cResponse::GetBodyRotation(r) => CommandResult::GetBodyRotation(r),
            I2cResponse::GetLegRadius(r) => CommandResult::GetLegRadius(r),
            I2cResponse::GetBatteryLevel(l) => CommandResult::GetBatteryLevel(l),
            I2cResponse::GetCalibration {
                leg,
                joint,
                kind,
                pulse,
            } => CommandResult::GetCalibrationFor(CalibrationDatum::new(
                pulse,
                CalibrationIndex { leg, joint, kind },
            )),
            I2cResponse::SetCalibration {
                leg,
                joint,
                kind,
                pulse: _,
                res,
            } => CommandResult::SetCalibrationDatum(SetResult {
                index: CalibrationIndex { leg, joint, kind },
                result: match res {
                    communication::SetRes::Ok => SetRes::Ok,
                    communication::SetRes::InvalidIndex => SetRes::InvalidIndex,
                    communication::SetRes::InvalidValue => SetRes::InvalidValue,
                    communication::SetRes::InvalidKind => SetRes::InvalidKind,
                    communication::SetRes::InvalidLeg => SetRes::InvalidLeg,
                    communication::SetRes::InvalidJoint => SetRes::InvalidJoint,
                },
            }),
        }
    }
}

impl Command {
    pub(crate) fn to_results(
        self,
        robot_state: &RobotState,
        result: Result<Vec<I2cResponse, 16>, CommandErr>,
    ) -> Vec<CommandResult, 16> {
        let mut v = Vec::new();
        let last_res = match (self, result) {
            (Command::NotifyAll, Ok(results)) => {
                for res in results {
                    let command_result = res.into();
                    if let Err(e) = v.push(command_result) {
                        log::error!("Cannot push {:?}: {:?}", command_result, e);
                    }
                }
                CommandResult::NotifyAll
            }
            (Command::NotifyAll, Err(_)) => CommandResult::NotifyAll,
            (Command::Sync, Ok(results)) => {
                for res in results {
                    let command_result = res.into();
                    if let Err(e) = v.push(command_result) {
                        log::error!("Cannot push {:?}: {:?}", command_result, e);
                    }
                }
                CommandResult::Sync
            }
            (Command::Sync, Err(_)) => CommandResult::Sync,
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
            (Command::GetCalibrationFor(_), Ok(_)) => {
                // Handled by Sync.
                return v;
            }
            (Command::GetCalibrationFor(index), Err(e)) => {
                log::error!("Error getting calibration for {:?}: {:?}", index, e);
                // Handled by Sync.
                return v;
            }
            (Command::SetCalibrationDatum(_, _), Ok(_)) => {
                // Handled by Sync.
                return v;
            }
            (Command::SetCalibrationDatum(_, index), Err(e)) => {
                log::error!("Error setting calibration for {:?}: {:?}", index, e);
                // Handled by Sync.
                return v;
            }
        };
        if let Err(e) = v.push(last_res) {
            log::error!("Cannot push {:?}: {:?}", last_res, e);
        }
        v
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Command {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Command::NotifyAll => defmt::write!(f, "NotifyAll"),
            Command::Sync => defmt::write!(f, "Sync"),
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
pub(crate) enum CommandResult {
    NotifyAll,
    Sync,
    GetBatteryLevel(u32),
    ChangeState(StateMachine),
    GetState(StateMachine),
    SetAnimationFactor(f32),
    GetAnimationFactor(f32),
    SetBodyTranslation(Translation3<f32>),
    GetBodyTranslation(Translation3<f32>),
    SetBodyRotation(UnitQuaternion<f32>),
    GetBodyRotation(UnitQuaternion<f32>),
    SetMotionVector(Vector3<f32>),
    GetMotionVector(Vector3<f32>),
    SetAngularVelocity(f32),
    GetAngularVelocity(f32),
    SetLegRadius(f32),
    GetLegRadius(f32),
    SetBatteryUpdateInterval(u32),
    GetBatteryUpdateInterval(u32),
    GetCalibrationFor(CalibrationDatum),
    SetCalibrationDatum(SetResult),
}

#[cfg(feature = "defmt")]
impl defmt::Format for CommandResult {
    fn format(&self, f: defmt::Formatter) {
        match self {
            CommandResult::NotifyAll => defmt::write!(f, "NotifyAll"),
            CommandResult::Sync => defmt::write!(f, "Sync"),
            CommandResult::GetBatteryLevel(l) => defmt::write!(f, "GetBatteryLevel({})", l),
            CommandResult::ChangeState(sm) => match sm {
                StateMachine::Paused => defmt::write!(f, "ChangeState(Paused)"),
                StateMachine::Homing => defmt::write!(f, "ChangeState(Homing)"),
                StateMachine::Calibrating => defmt::write!(f, "ChangeState(Calibrating)"),
                StateMachine::Looping => defmt::write!(f, "ChangeState(Looping)"),
                StateMachine::Exploring => defmt::write!(f, "ChangeState(Exploring)"),
            },
            CommandResult::GetState(sm) => match sm {
                StateMachine::Paused => defmt::write!(f, "GetState(Paused)"),
                StateMachine::Homing => defmt::write!(f, "GetState(Homing)"),
                StateMachine::Calibrating => defmt::write!(f, "GetState(Calibrating)"),
                StateMachine::Looping => defmt::write!(f, "GetState(Looping)"),
                StateMachine::Exploring => defmt::write!(f, "GetState(Exploring)"),
            },
            CommandResult::SetAnimationFactor(s) => defmt::write!(f, "SetAnimationFactor({})", s),
            CommandResult::GetAnimationFactor(s) => defmt::write!(f, "GetAnimationFactor({})", s),
            CommandResult::SetBodyTranslation(t) => {
                defmt::write!(f, "SetBodyTranslation({}, {}, {})", t.x, t.y, t.z)
            }
            CommandResult::GetBodyTranslation(t) => {
                defmt::write!(f, "GetBodyTranslation({}, {}, {})", t.x, t.y, t.z)
            }
            CommandResult::SetBodyRotation(r) => {
                defmt::write!(f, "SetBodyRotation({},{},{},{})", r.i, r.j, r.k, r.w)
            }
            CommandResult::GetBodyRotation(r) => {
                defmt::write!(f, "GetBodyRotation({},{},{},{})", r.i, r.j, r.k, r.w)
            }
            CommandResult::SetMotionVector(v) => {
                defmt::write!(f, "SetMotionVector({}, {}, {})", v.x, v.y, v.z)
            }
            CommandResult::GetMotionVector(v) => {
                defmt::write!(f, "GetMotionVector({}, {}, {})", v.x, v.y, v.z)
            }
            CommandResult::SetAngularVelocity(v) => defmt::write!(f, "SetAngularVelocity({})", v),
            CommandResult::GetAngularVelocity(v) => defmt::write!(f, "GetAngularVelocity({})", v),
            CommandResult::SetLegRadius(r) => defmt::write!(f, "SetLegRadius({})", r),
            CommandResult::GetLegRadius(r) => defmt::write!(f, "GetLegRadius({})", r),
            CommandResult::SetBatteryUpdateInterval(i) => {
                defmt::write!(f, "SetBatteryUpdateInterval({})", i)
            }
            CommandResult::GetBatteryUpdateInterval(i) => {
                defmt::write!(f, "GetBatteryUpdateInterval({})", i)
            }
            CommandResult::GetCalibrationFor(index) => {
                defmt::write!(f, "GetCalibrationFor({})", index)
            }
            CommandResult::SetCalibrationDatum(result) => {
                defmt::write!(f, "SetCalibrationDatum({:?})", result)
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
    pub(crate) battery_level: Option<UpdateKind>,
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
            battery_level: None,
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
            CommandResult::NotifyAll => None,
            CommandResult::Sync => None,
            CommandResult::GetBatteryLevel(_) => self.battery_level,
            CommandResult::ChangeState(_) | CommandResult::GetState(_) => self.state,
            CommandResult::SetAnimationFactor(_) | CommandResult::GetAnimationFactor(_) => {
                self.animation_factor
            }
            CommandResult::SetBodyTranslation(_) | CommandResult::GetBodyTranslation(_) => {
                self.body_translation
            }
            CommandResult::SetBodyRotation(_) | CommandResult::GetBodyRotation(_) => {
                self.body_rotation
            }
            CommandResult::SetMotionVector(_) | CommandResult::GetMotionVector(_) => {
                self.motion_vector
            }
            CommandResult::SetAngularVelocity(_) | CommandResult::GetAngularVelocity(_) => {
                self.angular_velocity
            }
            CommandResult::SetLegRadius(_) | CommandResult::GetLegRadius(_) => self.leg_radius,
            CommandResult::SetBatteryUpdateInterval(_)
            | CommandResult::GetBatteryUpdateInterval(_) => self.battery_interval,
            CommandResult::GetCalibrationFor(_) => self.get_calibration_result,
            CommandResult::SetCalibrationDatum(..) => self.set_calibration_result,
        }
    }
}

mod state_manager {
    use bluetooth_comms::log;
    use communication::I2cRequest;

    pub(crate) struct StateManager {
        should_update: [Option<I2cRequest>; 18],
    }

    impl StateManager {
        pub(crate) fn new() -> Self {
            Self {
                should_update: [None; 18],
            }
        }

        pub(super) fn insert(&mut self, req: I2cRequest) {
            log::info!("Inserting {:?} into state manager", req);
            let idx = match req {
                I2cRequest::ChangeState(_) => 0,
                I2cRequest::SetAnimationFactor(_) => 1,
                I2cRequest::GetAnimationFactor => 2,
                I2cRequest::SetAngularVelocity(_) => 3,
                I2cRequest::GetAngularVelocity => 4,
                I2cRequest::SetMotionVector(_) => 5,
                I2cRequest::GetMotionVector => 6,
                I2cRequest::SetBodyTranslation(_) => 7,
                I2cRequest::GetBodyTranslation => 8,
                I2cRequest::SetBodyRotation(_) => 9,
                I2cRequest::GetBodyRotation => 10,
                I2cRequest::SetLegRadius(_) => 11,
                I2cRequest::GetLegRadius => 12,
                I2cRequest::SetBatteryUpdateInterval(_) => 13,
                I2cRequest::GetBatteryLevel => 14,
                I2cRequest::SetCalibration { .. } => 15,
                I2cRequest::GetCalibration { .. } => 16,
                I2cRequest::GetState => 17,
            };
            self.should_update[idx] = Some(req);
        }

        pub(super) fn clear(&mut self) {
            log::info!("Clearing state manager");
            self.should_update = [None; 18];
        }

        pub(super) fn iter(&self) -> impl Iterator<Item = I2cRequest> + '_ {
            Iter {
                iter: self.should_update.iter(),
            }
        }
    }

    struct Iter<'a> {
        iter: core::slice::Iter<'a, Option<I2cRequest>>,
    }

    impl<'a> Iterator for Iter<'a> {
        type Item = I2cRequest;

        fn next(&mut self) -> Option<Self::Item> {
            self.iter.find_map(|req| req.as_ref().copied())
        }
    }
}
pub(crate) use state_manager::StateManager;

pub(crate) async fn handle_command(
    robot_state: &mut RobotState,
    state_manager: &mut StateManager,
    request_tx: CriticalSender<I2cRequest>,
    response_rx: CriticalReceiver<Result<I2cResponse, CommsError>>,
    command: Command,
) -> Result<Vec<I2cResponse, 16>, CommandErr> {
    log::info!("Handling {}", command);
    let mut vec = Vec::new();
    match command {
        Command::NotifyAll => {
            let reqs = [
                I2cRequest::GetBatteryLevel,
                I2cRequest::GetState,
                I2cRequest::GetAngularVelocity,
                I2cRequest::GetAnimationFactor,
                I2cRequest::GetMotionVector,
                I2cRequest::GetBodyTranslation,
                I2cRequest::GetBodyRotation,
                I2cRequest::GetLegRadius,
            ];
            // TODO Get all calibrations. Maybe handle via another channel
            // rather than a vec?
            for req in reqs {
                request_tx.send(req).await;
                let res = response_rx.receive().await;
                match res {
                    Ok(res) => {
                        if let Err(e) = vec.push(res) {
                            log::panic!("Unexpected error: {:?}", e);
                        }
                    }
                    Err(e) => {
                        log::error!("Error receiving response for {:?}: {:?}", req, e);
                    }
                }
            }
        }
        Command::Sync => {
            for req in state_manager.iter() {
                log::info!("Sending update {:?}", req);
                request_tx.send(req).await;
                if req.is_get() {
                    let res = response_rx.receive().await;
                    let res = match res {
                        Ok(res) => res,
                        Err(e) => {
                            log::error!("Error receiving response: {:?}", e);
                            continue;
                        }
                    };
                    if let Err(e) = vec.push(res) {
                        log::panic!("Unexpected error: {:?}", e);
                    }
                }
            }

            state_manager.clear();
        }
        Command::ChangeState(sm) => {
            if robot_state.state_machine != sm {
                robot_state.state_machine = sm;
                state_manager.insert(I2cRequest::ChangeState(sm));
            }
        }
        Command::SetAnimationFactor(val) => {
            if robot_state.animation_factor != val {
                if (MIN_SPEED..=MAX_SPEED).contains(&val) {
                    robot_state.animation_factor = val;
                    state_manager.insert(I2cRequest::SetAnimationFactor(val));
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
                    state_manager.insert(I2cRequest::SetBodyTranslation(val));
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
                    state_manager.insert(I2cRequest::SetBodyRotation(val));
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
                    state_manager.insert(I2cRequest::SetMotionVector(val));
                } else {
                    return Err(CommandErr::InvalidFloat);
                }
            }
        }
        Command::SetAngularVelocity(val) => {
            if robot_state.angular_velocity != val {
                if (MIN_AVEL..=MAX_AVEL).contains(&val) {
                    robot_state.angular_velocity = val;
                    state_manager.insert(I2cRequest::SetAngularVelocity(val));
                } else {
                    return Err(CommandErr::InvalidFloat);
                }
            }
        }
        Command::SetLegRadius(val) => {
            if robot_state.leg_radius != val {
                if (MIN_LRAD..=MAX_LRAD).contains(&val) {
                    robot_state.leg_radius = val;
                    state_manager.insert(I2cRequest::SetLegRadius(val));
                } else {
                    return Err(CommandErr::InvalidFloat);
                }
            }
        }
        Command::SetBatteryUpdateInterval(val) => {
            if robot_state.battery_update_interval_ms != val {
                if (MIN_UPIVAL..=MAX_UPIVAL).contains(&val) {
                    robot_state.battery_update_interval_ms = val;
                    state_manager.insert(I2cRequest::SetBatteryUpdateInterval(val));
                } else {
                    return Err(CommandErr::InvalidFloat);
                }
            }
        }
        Command::GetCalibrationFor(index) => {
            // check index
            if index.leg > 6
                || index.joint > 3
                || index.kind > (if index.joint == 2 { 4 } else { 3 })
            {
                return Err(CommandErr::InvalidIndex);
            }
            state_manager.insert(I2cRequest::GetCalibration {
                leg: index.leg,
                joint: index.joint,
                kind: index.kind,
            });
        }
        Command::SetCalibrationDatum(pulse, index) => {
            // check index
            if index.leg > 6
                || index.joint > 3
                || index.kind > (if index.joint == 2 { 4 } else { 3 })
            {
                return Err(CommandErr::InvalidIndex);
            }
            robot_state.joint_mut(index.leg, index.joint).unwrap().cal[index.kind as usize]
                .pulse_ms = pulse;
            state_manager.insert(I2cRequest::SetCalibration {
                leg: index.leg,
                joint: index.joint,
                kind: index.kind,
                pulse,
            });
        }
    }
    log::info!("Done handling {}", command);

    Ok(vec)
}
