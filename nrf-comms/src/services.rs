use nrf_softdevice::ble::gatt_server::{IndicateValueError, NotifyValueError};
use nrf_softdevice::ble::Connection;
use state::RobotState;

use crate::command::{CommandUpdate, UpdateKind};
use crate::server::Server;
use crate::wrappers::{Translation, UQuaternion, Vector, F32, SM};
use crate::{log, Command};

#[nrf_softdevice::gatt_service(uuid = "180f")]
pub(crate) struct BatteryService {
    #[characteristic(uuid = "2a19", read, notify)]
    pub(crate) battery_level: u8,
}

#[nrf_softdevice::gatt_service(uuid = "77144c32-0ed7-469b-a3d3-0f2c9157333a")]
pub(crate) struct ControllerService {
    #[characteristic(uuid = "b0115f52-c6fd-4ea4-94cf-21626b9e3469", write)]
    pub(crate) sync: bool,
    #[characteristic(
        uuid = "f125c904-a0e2-4885-817c-55d7463630db",
        read,
        write,
        notify,
        indicate
    )]
    pub(crate) state: SM,
    #[characteristic(
        uuid = "7566bd93-3712-4850-8868-88ce30f6acc1",
        read,
        write,
        notify,
        indicate
    )]
    pub(crate) animation_factor: F32,
    #[characteristic(
        uuid = "6be80625-e69b-418e-bf01-9abc617cdd9f",
        read,
        write,
        notify,
        indicate
    )]
    pub(crate) angular_velocity: F32,
    #[characteristic(
        uuid = "77d6f220-7057-4dc6-8746-8a23b06e53d6",
        read,
        write,
        notify,
        indicate
    )]
    pub(crate) motion_vector: Vector,
    #[characteristic(
        uuid = "cde4ce10-edc2-44af-bd14-60865d30f2b6",
        read,
        write,
        notify,
        indicate
    )]
    pub(crate) body_translation: Translation,
    #[characteristic(
        uuid = "ccfb948a-3421-47ed-a0c0-b84f7d307027",
        read,
        write,
        notify,
        indicate
    )]
    pub(crate) body_rotation: UQuaternion,
    #[characteristic(
        uuid = "2735f1d0-b944-4efe-960c-10380d061052",
        read,
        write,
        notify,
        indicate
    )]
    pub(crate) leg_radius: F32,
    #[characteristic(
        uuid = "d007632f-10e5-427a-b158-482aeb48b90e",
        read,
        write,
        notify,
        indicate
    )]
    pub(crate) battery_update_interval_ms: u32,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum UpdateError {
    Notify(NotifyValueError),
    Indicate(IndicateValueError),
}

impl From<NotifyValueError> for UpdateError {
    fn from(err: NotifyValueError) -> Self {
        Self::Notify(err)
    }
}

impl From<IndicateValueError> for UpdateError {
    fn from(err: IndicateValueError) -> Self {
        Self::Indicate(err)
    }
}

pub(crate) fn update(
    server: &'static Server,
    conn: &Connection,
    command_update: &CommandUpdate,
    command: Command,
    robot_state: &RobotState,
) -> Result<(), UpdateError> {
    match (command, command_update.get(command)) {
        (Command::Sync(_), Some(UpdateKind::Notify)) => {
            server
                .bas()
                .battery_level_notify(conn, &(robot_state.battery_level as u8))?;
        }
        (Command::ChangeState(_), Some(UpdateKind::Notify)) => server
            .controller()
            .state_notify(conn, &SM(robot_state.state_machine))?,
        (Command::ChangeState(_), Some(UpdateKind::Indicate)) => server
            .controller()
            .state_indicate(conn, &SM(robot_state.state_machine))?,
        (Command::SetAnimationFactor(_), Some(UpdateKind::Notify)) => server
            .controller()
            .animation_factor_notify(conn, &F32(robot_state.animation_factor))?,
        (Command::SetAnimationFactor(_), Some(UpdateKind::Indicate)) => server
            .controller()
            .animation_factor_indicate(conn, &F32(robot_state.animation_factor))?,
        (Command::SetBodyTranslation(_), Some(UpdateKind::Notify)) => server
            .controller()
            .body_translation_notify(conn, &Translation(robot_state.body_translation))?,
        (Command::SetBodyTranslation(_), Some(UpdateKind::Indicate)) => server
            .controller()
            .body_translation_indicate(conn, &Translation(robot_state.body_translation))?,
        (Command::SetBodyRotation(_), Some(UpdateKind::Notify)) => server
            .controller()
            .body_rotation_notify(conn, &UQuaternion(robot_state.body_rotation))?,
        (Command::SetBodyRotation(_), Some(UpdateKind::Indicate)) => server
            .controller()
            .body_rotation_indicate(conn, &UQuaternion(robot_state.body_rotation))?,
        (Command::SetMotionVector(_), Some(UpdateKind::Notify)) => server
            .controller()
            .motion_vector_notify(conn, &Vector(robot_state.motion_vector))?,
        (Command::SetMotionVector(_), Some(UpdateKind::Indicate)) => server
            .controller()
            .motion_vector_indicate(conn, &Vector(robot_state.motion_vector))?,
        (Command::SetAngularVelocity(_), Some(UpdateKind::Notify)) => server
            .controller()
            .angular_velocity_notify(conn, &F32(robot_state.angular_velocity))?,
        (Command::SetAngularVelocity(_), Some(UpdateKind::Indicate)) => server
            .controller()
            .angular_velocity_indicate(conn, &F32(robot_state.angular_velocity))?,
        (Command::SetLegRadius(_), Some(UpdateKind::Notify)) => server
            .controller()
            .leg_radius_notify(conn, &F32(robot_state.leg_radius))?,
        (Command::SetLegRadius(_), Some(UpdateKind::Indicate)) => server
            .controller()
            .leg_radius_indicate(conn, &F32(robot_state.leg_radius))?,
        (_, None) => {}
        (command, update) => log::warn!("Unexpected combination: {}, {}", command, update),
    }
    Ok(())
}
