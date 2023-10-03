use nrf_softdevice::ble::gatt_server::{IndicateValueError, NotifyValueError};
use nrf_softdevice::ble::{Connection, FixedGattValue};

use crate::command::{CalibrationIndex, CommandResult, CommandUpdate, UpdateKind};
use crate::log;
use crate::server::Server;
use crate::wrappers::{Translation, UQuaternion, Vector, F32, SM};

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

    #[characteristic(uuid = "22b55af4-32f7-41bc-88f8-021e5dce9f60", write)]
    pub(crate) get_calibation_for: CalibrationIndex,

    #[characteristic(uuid = "be20dff1-36a5-49de-a7c8-cec51bae2c4e", notify)]
    pub(crate) get_calibration_result: CalibrationDatum,

    #[characteristic(uuid = "f34c2b97-e619-4ed8-ae1b-3e623855bb8e", write)]
    pub(crate) set_calibration_datum: CalibrationDatum,

    #[characteristic(uuid = "a9b70415-bb6c-4aca-aeb3-ac93c6087910", notify)]
    pub(crate) set_calibration_result: SetResult,
}

impl FixedGattValue for CalibrationIndex {
    const SIZE: usize = core::mem::size_of::<CalibrationIndex>();

    fn from_gatt(data: &[u8]) -> Self {
        Self {
            leg: data[0],
            joint: data[1],
            kind: data[2],
        }
    }

    fn to_gatt(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                &self as *const _ as *const u8,
                core::mem::size_of::<CalibrationIndex>(),
            )
        }
    }
}

#[derive(Copy, Clone)]
#[repr(C, packed(4))]
pub(crate) struct CalibrationDatum {
    pub(crate) value: F32,
    pub(crate) index: CalibrationIndex,
    _packed: u8,
}

impl CalibrationDatum {
    pub(crate) fn new(value: f32, index: CalibrationIndex) -> Self {
        Self {
            value: F32(value),
            index,
            _packed: 0,
        }
    }
}

const _: () = assert!(core::mem::size_of::<CalibrationDatum>() == 8);
const _: () = assert!(core::mem::align_of::<CalibrationDatum>() == 4);

impl FixedGattValue for CalibrationDatum {
    const SIZE: usize = core::mem::size_of::<CalibrationDatum>();

    fn from_gatt(data: &[u8]) -> Self {
        Self {
            value: F32::from_gatt(&data[0..4]),
            index: CalibrationIndex::from_gatt(&data[4..7]),
            _packed: 0,
        }
    }

    fn to_gatt(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                self as *const _ as *const u8,
                core::mem::size_of::<CalibrationDatum>(),
            )
        }
    }
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub(crate) enum SetRes {
    Ok = 0,
    InvalidIndex = 1,
    InvalidValue = 2,
    InvalidKind = 3,
    InvalidLeg = 4,
    InvalidJoint = 5,
}

#[derive(Copy, Clone)]
#[repr(C, packed(4))]
pub(crate) struct SetResult {
    pub(crate) index: CalibrationIndex,
    pub(crate) result: SetRes,
}

impl FixedGattValue for SetResult {
    const SIZE: usize = core::mem::size_of::<SetResult>();

    fn from_gatt(data: &[u8]) -> Self {
        Self {
            index: CalibrationIndex::from_gatt(&data[0..3]),
            result: match data[3] {
                0 => SetRes::Ok,
                1 => SetRes::InvalidIndex,
                2 => SetRes::InvalidValue,
                3 => SetRes::InvalidKind,
                4 => SetRes::InvalidLeg,
                5 => SetRes::InvalidJoint,
                _ => unreachable!(),
            },
        }
    }

    fn to_gatt(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                self as *const _ as *const u8,
                core::mem::size_of::<SetResult>(),
            )
        }
    }
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
    match (command_result, command_update.get(command_result)) {
        (CommandResult::Sync(battery_level), Some(UpdateKind::Notify)) => {
            server.bas().battery_level_notify(conn, &(battery_level))?;
        }
        (CommandResult::ChangeState(state_machine), Some(UpdateKind::Notify)) => {
            server.controller().state_notify(conn, &SM(state_machine))?
        }
        (CommandResult::ChangeState(state_machine), Some(UpdateKind::Indicate)) => server
            .controller()
            .state_indicate(conn, &SM(state_machine))?,
        (CommandResult::SetAnimationFactor(animation_factor), Some(UpdateKind::Notify)) => server
            .controller()
            .animation_factor_notify(conn, &F32(animation_factor))?,
        (CommandResult::SetAnimationFactor(animation_factor), Some(UpdateKind::Indicate)) => server
            .controller()
            .animation_factor_indicate(conn, &F32(animation_factor))?,
        (CommandResult::SetBodyTranslation(body_translation), Some(UpdateKind::Notify)) => server
            .controller()
            .body_translation_notify(conn, &Translation(body_translation))?,
        (CommandResult::SetBodyTranslation(body_translation), Some(UpdateKind::Indicate)) => server
            .controller()
            .body_translation_indicate(conn, &Translation(body_translation))?,
        (CommandResult::SetBodyRotation(body_rotation), Some(UpdateKind::Notify)) => server
            .controller()
            .body_rotation_notify(conn, &UQuaternion(body_rotation))?,
        (CommandResult::SetBodyRotation(body_rotation), Some(UpdateKind::Indicate)) => server
            .controller()
            .body_rotation_indicate(conn, &UQuaternion(body_rotation))?,
        (CommandResult::SetMotionVector(motion_vector), Some(UpdateKind::Notify)) => server
            .controller()
            .motion_vector_notify(conn, &Vector(motion_vector))?,
        (CommandResult::SetMotionVector(motion_vector), Some(UpdateKind::Indicate)) => server
            .controller()
            .motion_vector_indicate(conn, &Vector(motion_vector))?,
        (CommandResult::SetAngularVelocity(angular_velocity), Some(UpdateKind::Notify)) => server
            .controller()
            .angular_velocity_notify(conn, &F32(angular_velocity))?,
        (CommandResult::SetAngularVelocity(angular_velocity), Some(UpdateKind::Indicate)) => server
            .controller()
            .angular_velocity_indicate(conn, &F32(angular_velocity))?,
        (CommandResult::SetLegRadius(leg_radius), Some(UpdateKind::Notify)) => server
            .controller()
            .leg_radius_notify(conn, &F32(leg_radius))?,
        (CommandResult::SetLegRadius(leg_radius), Some(UpdateKind::Indicate)) => server
            .controller()
            .leg_radius_indicate(conn, &F32(leg_radius))?,
        (CommandResult::GetCalibrationFor(cal_datum), Some(UpdateKind::Notify)) => server
            .controller()
            .get_calibration_result_notify(conn, &cal_datum)?,
        (CommandResult::SetCalibrationDatum(set_result), Some(UpdateKind::Notify)) => server
            .controller()
            .set_calibration_result_notify(conn, &set_result)?,
        (_, None) => {}
        (command, update) => log::warn!("Unexpected combination: {}, {}", command, update),
    }
    Ok(())
}
