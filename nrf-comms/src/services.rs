use nrf_softdevice::ble::gatt_server::{IndicateValueError, NotifyValueError};
use nrf_softdevice::ble::{Connection, FixedGattValue};

use bluetooth_comms::wrappers::{Translation, UQuaternion, Vector, SM};
use bluetooth_comms::{CalibrationDatum, CalibrationIndex, SetResult, TestableFixedGattValue};

use crate::command::{CommandResult, CommandUpdate, UpdateKind};
use crate::log;
use crate::server::Server;

pub(crate) struct Wrapper<T>(pub(crate) T);

impl<T> FixedGattValue for Wrapper<T>
where
    T: TestableFixedGattValue,
{
    const SIZE: usize = <T as TestableFixedGattValue>::SIZE;

    fn from_gatt(data: &[u8]) -> Self {
        Wrapper(<T as TestableFixedGattValue>::from_gatt(data))
    }

    fn to_gatt(&self) -> &[u8] {
        <T as TestableFixedGattValue>::to_gatt(&self.0)
    }
}

#[nrf_softdevice::gatt_service(uuid = "180f")]
pub(crate) struct BatteryService {
    #[characteristic(uuid = "2a19", read, notify)]
    pub(crate) battery_level: u8,
}

#[nrf_softdevice::gatt_service(uuid = "77144c32-0ed7-469b-a3d3-0f2c9157333a")]
pub(crate) struct ControllerService {
    #[characteristic(uuid = "3f6b77c2-0205-4a77-a809-2479b9533664", write)]
    pub(crate) notify_all: bool,

    #[characteristic(uuid = "b0115f52-c6fd-4ea4-94cf-21626b9e3469", write)]
    pub(crate) sync: bool,

    #[characteristic(
        uuid = "f125c904-a0e2-4885-817c-55d7463630db",
        read,
        write,
        notify,
        indicate
    )]
    pub(crate) state: Wrapper<SM>,

    #[characteristic(
        uuid = "7566bd93-3712-4850-8868-88ce30f6acc1",
        read,
        write,
        notify,
        indicate
    )]
    pub(crate) animation_factor: Wrapper<f32>,

    #[characteristic(
        uuid = "6be80625-e69b-418e-bf01-9abc617cdd9f",
        read,
        write,
        notify,
        indicate
    )]
    pub(crate) angular_velocity: Wrapper<f32>,

    #[characteristic(
        uuid = "77d6f220-7057-4dc6-8746-8a23b06e53d6",
        read,
        write,
        notify,
        indicate
    )]
    pub(crate) motion_vector: Wrapper<Vector>,

    #[characteristic(
        uuid = "cde4ce10-edc2-44af-bd14-60865d30f2b6",
        read,
        write,
        notify,
        indicate
    )]
    pub(crate) body_translation: Wrapper<Translation>,

    #[characteristic(
        uuid = "ccfb948a-3421-47ed-a0c0-b84f7d307027",
        read,
        write,
        notify,
        indicate
    )]
    pub(crate) body_rotation: Wrapper<UQuaternion>,

    #[characteristic(
        uuid = "2735f1d0-b944-4efe-960c-10380d061052",
        read,
        write,
        notify,
        indicate
    )]
    pub(crate) leg_radius: Wrapper<f32>,

    #[characteristic(
        uuid = "d007632f-10e5-427a-b158-482aeb48b90e",
        read,
        write,
        notify,
        indicate
    )]
    pub(crate) battery_update_interval_ms: u32,

    #[characteristic(uuid = "22b55af4-32f7-41bc-88f8-021e5dce9f60", write)]
    pub(crate) get_calibation_for: Wrapper<CalibrationIndex>,

    #[characteristic(uuid = "be20dff1-36a5-49de-a7c8-cec51bae2c4e", notify)]
    pub(crate) get_calibration_result: Wrapper<CalibrationDatum>,

    #[characteristic(uuid = "f34c2b97-e619-4ed8-ae1b-3e623855bb8e", write)]
    pub(crate) set_calibration_datum: Wrapper<CalibrationDatum>,

    #[characteristic(uuid = "a9b70415-bb6c-4aca-aeb3-ac93c6087910", notify)]
    pub(crate) set_calibration_result: Wrapper<SetResult>,
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
    command_result: CommandResult,
) -> Result<(), UpdateError> {
    log::info!(
        "Sending update for {:?}: {:?}",
        command_result,
        command_update.get(command_result)
    );
    match (command_result, command_update.get(command_result)) {
        (CommandResult::NotifyAll, _) => {
            if let Some(UpdateKind::Notify) = command_update.battery_level {
                let _ = server
                    .bas()
                    .battery_level_notify(conn, &server.bas().battery_level_get().unwrap());
            }
            match command_update.state {
                Some(UpdateKind::Notify) => {
                    let _ = server
                        .controller()
                        .state_notify(conn, &server.controller().state_get().unwrap());
                }
                Some(UpdateKind::Indicate) => {
                    let _ = server
                        .controller()
                        .state_indicate(conn, &server.controller().state_get().unwrap());
                }
                None => {}
            }
            match command_update.animation_factor {
                Some(UpdateKind::Notify) => {
                    let _ = server.controller().animation_factor_notify(
                        conn,
                        &server.controller().animation_factor_get().unwrap(),
                    );
                }
                Some(UpdateKind::Indicate) => {
                    let _ = server.controller().animation_factor_indicate(
                        conn,
                        &server.controller().animation_factor_get().unwrap(),
                    );
                }
                None => {}
            }
            match command_update.angular_velocity {
                Some(UpdateKind::Notify) => {
                    let _ = server.controller().angular_velocity_notify(
                        conn,
                        &server.controller().angular_velocity_get().unwrap(),
                    );
                }
                Some(UpdateKind::Indicate) => {
                    let _ = server.controller().angular_velocity_indicate(
                        conn,
                        &server.controller().angular_velocity_get().unwrap(),
                    );
                }
                None => {}
            }
            match command_update.motion_vector {
                Some(UpdateKind::Notify) => {
                    let _ = server.controller().motion_vector_notify(
                        conn,
                        &server.controller().motion_vector_get().unwrap(),
                    );
                }
                Some(UpdateKind::Indicate) => {
                    let _ = server.controller().motion_vector_indicate(
                        conn,
                        &server.controller().motion_vector_get().unwrap(),
                    );
                }
                None => {}
            }
            match command_update.body_translation {
                Some(UpdateKind::Notify) => {
                    let _ = server.controller().body_translation_notify(
                        conn,
                        &server.controller().body_translation_get().unwrap(),
                    );
                }
                Some(UpdateKind::Indicate) => {
                    let _ = server.controller().body_translation_indicate(
                        conn,
                        &server.controller().body_translation_get().unwrap(),
                    );
                }
                None => {}
            }
            match command_update.body_rotation {
                Some(UpdateKind::Notify) => {
                    let _ = server.controller().body_rotation_notify(
                        conn,
                        &server.controller().body_rotation_get().unwrap(),
                    );
                }
                Some(UpdateKind::Indicate) => {
                    let _ = server.controller().body_rotation_indicate(
                        conn,
                        &server.controller().body_rotation_get().unwrap(),
                    );
                }
                None => {}
            }
            match command_update.leg_radius {
                Some(UpdateKind::Notify) => {
                    let _ = server
                        .controller()
                        .leg_radius_notify(conn, &server.controller().leg_radius_get().unwrap());
                }
                Some(UpdateKind::Indicate) => {
                    let _ = server
                        .controller()
                        .leg_radius_indicate(conn, &server.controller().leg_radius_get().unwrap());
                }
                None => {}
            }
        }
        (CommandResult::Sync, _) => {}
        (
            CommandResult::ChangeState(state_machine) | CommandResult::GetState(state_machine),
            Some(UpdateKind::Notify),
        ) => server
            .controller()
            .state_notify(conn, &Wrapper(SM(state_machine)))?,
        (
            CommandResult::ChangeState(state_machine) | CommandResult::GetState(state_machine),
            Some(UpdateKind::Indicate),
        ) => server
            .controller()
            .state_indicate(conn, &Wrapper(SM(state_machine)))?,
        (
            CommandResult::SetAnimationFactor(animation_factor)
            | CommandResult::GetAnimationFactor(animation_factor),
            Some(UpdateKind::Notify),
        ) => server
            .controller()
            .animation_factor_notify(conn, &Wrapper(animation_factor))?,
        (
            CommandResult::SetAnimationFactor(animation_factor)
            | CommandResult::GetAnimationFactor(animation_factor),
            Some(UpdateKind::Indicate),
        ) => server
            .controller()
            .animation_factor_indicate(conn, &Wrapper(animation_factor))?,
        (
            CommandResult::SetBodyTranslation(body_translation)
            | CommandResult::GetBodyTranslation(body_translation),
            Some(UpdateKind::Notify),
        ) => server
            .controller()
            .body_translation_notify(conn, &Wrapper(Translation(body_translation)))?,
        (
            CommandResult::SetBodyTranslation(body_translation)
            | CommandResult::GetBodyTranslation(body_translation),
            Some(UpdateKind::Indicate),
        ) => server
            .controller()
            .body_translation_indicate(conn, &Wrapper(Translation(body_translation)))?,
        (
            CommandResult::SetBodyRotation(body_rotation)
            | CommandResult::GetBodyRotation(body_rotation),
            Some(UpdateKind::Notify),
        ) => server
            .controller()
            .body_rotation_notify(conn, &Wrapper(UQuaternion(body_rotation)))?,
        (
            CommandResult::SetBodyRotation(body_rotation)
            | CommandResult::GetBodyRotation(body_rotation),
            Some(UpdateKind::Indicate),
        ) => server
            .controller()
            .body_rotation_indicate(conn, &Wrapper(UQuaternion(body_rotation)))?,
        (
            CommandResult::SetMotionVector(motion_vector)
            | CommandResult::GetMotionVector(motion_vector),
            Some(UpdateKind::Notify),
        ) => server
            .controller()
            .motion_vector_notify(conn, &Wrapper(Vector(motion_vector)))?,
        (
            CommandResult::SetMotionVector(motion_vector)
            | CommandResult::GetMotionVector(motion_vector),
            Some(UpdateKind::Indicate),
        ) => server
            .controller()
            .motion_vector_indicate(conn, &Wrapper(Vector(motion_vector)))?,
        (
            CommandResult::SetAngularVelocity(angular_velocity)
            | CommandResult::GetAngularVelocity(angular_velocity),
            Some(UpdateKind::Notify),
        ) => server
            .controller()
            .angular_velocity_notify(conn, &Wrapper(angular_velocity))?,
        (
            CommandResult::SetAngularVelocity(angular_velocity)
            | CommandResult::GetAngularVelocity(angular_velocity),
            Some(UpdateKind::Indicate),
        ) => server
            .controller()
            .angular_velocity_indicate(conn, &Wrapper(angular_velocity))?,
        (
            CommandResult::SetLegRadius(leg_radius) | CommandResult::GetLegRadius(leg_radius),
            Some(UpdateKind::Notify),
        ) => server
            .controller()
            .leg_radius_notify(conn, &Wrapper(leg_radius))?,
        (
            CommandResult::SetLegRadius(leg_radius) | CommandResult::GetLegRadius(leg_radius),
            Some(UpdateKind::Indicate),
        ) => server
            .controller()
            .leg_radius_indicate(conn, &Wrapper(leg_radius))?,
        (CommandResult::GetCalibrationFor(cal_datum), Some(UpdateKind::Notify)) => server
            .controller()
            .get_calibration_result_notify(conn, &Wrapper(cal_datum))?,
        (CommandResult::SetCalibrationDatum(set_result), Some(UpdateKind::Notify)) => server
            .controller()
            .set_calibration_result_notify(conn, &Wrapper(set_result))?,
        (_, None) => {}
        (command, update) => log::warn!("Unexpected combination: {}, {}", command, update),
    }
    log::info!(
        "Update sent for {:?}: {:?}",
        command_result,
        command_update.get(command_result)
    );
    Ok(())
}
