use core::mem::MaybeUninit;

use crate::log;
use nalgebra::{Const, Quaternion, SVector, Translation3, UnitQuaternion, Vector3};
use nrf_softdevice::ble::FixedGattValue;
use state::StateMachine;

fn vector_from_gatt<const SIZE: usize>(data: &[u8]) -> SVector<f32, SIZE> {
    assert!(data.len() >= core::mem::size_of::<f32>() * SIZE);
    let mut v = SVector::<MaybeUninit<f32>, SIZE>::uninit(Const::<SIZE>, Const::<1>);
    unsafe {
        // SAFETY: Sizes check is done in the assert above. Storage is properly aligned for f32
        // dest and copy is being done as array of u8's.
        core::ptr::copy_nonoverlapping(
            data.as_ptr(),
            v.as_mut_slice().as_mut_ptr() as *mut u8,
            SIZE,
        );
        // SAFETY: The array is initialized.
        v.assume_init()
    }
}

const ELEMS_3: usize = 3;
const ELEMS_4: usize = 4;

#[derive(Copy, Clone)]
pub(crate) struct Translation(pub(crate) Translation3<f32>);

impl FixedGattValue for Translation {
    const SIZE: usize = core::mem::size_of::<f32>() * ELEMS_3;

    fn from_gatt(data: &[u8]) -> Self {
        let data = vector_from_gatt::<{ ELEMS_3 }>(data);
        Self(Translation3::from(data))
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
#[repr(transparent)]
pub(crate) struct F32(pub(crate) f32);
impl FixedGattValue for F32 {
    const SIZE: usize = core::mem::size_of::<f32>();

    fn from_gatt(data: &[u8]) -> Self {
        assert!(data.len() >= Self::SIZE);

        // Safety: Ensuring proper alignment and f32 is Copy.
        F32(unsafe { core::ptr::read_unaligned(data.as_ptr() as *const f32) })
    }

    fn to_gatt(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(self as *const _ as *const u8, core::mem::size_of::<f32>())
        }
    }
}

#[derive(Copy, Clone)]
pub(crate) struct Vector(pub(crate) Vector3<f32>);

impl FixedGattValue for Vector {
    const SIZE: usize = core::mem::size_of::<f32>() * ELEMS_3;

    fn from_gatt(data: &[u8]) -> Self {
        let data = vector_from_gatt::<{ ELEMS_3 }>(data);
        Self(data)
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
    const SIZE: usize = core::mem::size_of::<f32>() * ELEMS_4;

    fn from_gatt(data: &[u8]) -> Self {
        let data = vector_from_gatt::<ELEMS_4>(data);
        Self(UnitQuaternion::new_normalize(Quaternion::from_vector(data)))
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
