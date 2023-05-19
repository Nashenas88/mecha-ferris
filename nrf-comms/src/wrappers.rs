use crate::log;
use nalgebra::{Quaternion, Translation3, UnitQuaternion, Vector3};
use nrf_softdevice::ble::FixedGattValue;
use state::StateMachine;

#[derive(Copy, Clone)]
pub(crate) struct Translation(pub(crate) Translation3<f32>);

impl FixedGattValue for Translation {
    const SIZE: usize = core::mem::size_of::<f32>() * 3;

    fn from_gatt(data: &[u8]) -> Self {
        log::info!("Vector::from_gatt({:?})", data);
        let data: &[f32] = unsafe { core::slice::from_raw_parts(data.as_ptr() as *const f32, 3) };
        log::info!("Vector::from_gatt data({:?})", data);
        Self(Translation3::new(data[0], data[1], data[2]))
    }

    fn to_gatt(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                self.0.vector.as_slice().as_ptr() as *const u8,
                core::mem::size_of::<f32>() * 3,
            )
        }
    }
}

#[derive(Copy, Clone)]
pub(crate) struct F32(pub(crate) f32);
impl FixedGattValue for F32 {
    const SIZE: usize = core::mem::size_of::<f32>();

    fn from_gatt(data: &[u8]) -> Self {
        log::info!("F32::from_gatt({:?})", data);
        let data: &[f32] = unsafe { core::slice::from_raw_parts(data.as_ptr() as *const f32, 1) };
        log::info!("F32::from_gatt data({:?})", data);
        Self(data[0])
    }

    fn to_gatt(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                &self.0 as *const _ as *const u8,
                core::mem::size_of::<f32>(),
            )
        }
    }
}

#[derive(Copy, Clone)]
pub(crate) struct Vector(pub(crate) Vector3<f32>);

impl FixedGattValue for Vector {
    const SIZE: usize = core::mem::size_of::<f32>() * 3;

    fn from_gatt(data: &[u8]) -> Self {
        log::info!("Vector::from_gatt({:?})", data);
        let data: &[f32] = unsafe { core::slice::from_raw_parts(data.as_ptr() as *const f32, 3) };
        log::info!("Vector::from_gatt data({:?})", data);
        Self(Vector3::new(data[0], data[1], data[2]))
    }

    fn to_gatt(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                self.0.data.as_slice().as_ptr() as *const u8,
                core::mem::size_of::<f32>() * 3,
            )
        }
    }
}

#[derive(Copy, Clone)]
pub(crate) struct UQuaternion(pub(crate) UnitQuaternion<f32>);

impl FixedGattValue for UQuaternion {
    const SIZE: usize = core::mem::size_of::<f32>() * 4;

    fn from_gatt(data: &[u8]) -> Self {
        let data: &[f32] = unsafe { core::slice::from_raw_parts(data.as_ptr() as *const f32, 4) };
        Self(UnitQuaternion::new_normalize(Quaternion::new(
            data[0], data[1], data[2], data[3],
        )))
    }

    fn to_gatt(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                self.0.as_ref().coords.as_slice().as_ptr() as *const u8,
                core::mem::size_of::<f32>() * 4,
            )
        }
    }
}

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct SM(pub(crate) StateMachine);

impl FixedGattValue for SM {
    const SIZE: usize = 1;

    fn from_gatt(data: &[u8]) -> Self {
        let sm = match u8::from_gatt(data).try_into() {
            Ok(sm) => sm,
            Err(e) => log::panic!("bad data for StateMachine: {:?}", e),
        };
        SM(sm)
    }

    fn to_gatt(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(&self.0 as *const StateMachine as *const u8, Self::SIZE)
        }
    }
}
