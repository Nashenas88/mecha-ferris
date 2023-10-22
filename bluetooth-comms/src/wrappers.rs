use crate::log;
use crate::TestableFixedGattValue;
use core::mem::MaybeUninit;
pub use nalgebra::{Const, Quaternion, SVector, Translation3, UnitQuaternion, Vector3};
pub use state::StateMachine;

fn vector_from_gatt<const SIZE: usize>(data: &[u8]) -> SVector<f32, SIZE> {
    assert!(data.len() >= core::mem::size_of::<f32>() * SIZE);
    let mut v = SVector::<MaybeUninit<f32>, SIZE>::uninit(Const::<SIZE>, Const::<1>);
    unsafe {
        // SAFETY: Sizes check is done in the assert above. Storage is properly aligned for f32
        // dest and copy is being done as array of u8's.
        core::ptr::copy_nonoverlapping(
            data.as_ptr(),
            v.as_mut_slice().as_mut_ptr() as *mut u8,
            SIZE * 4,
        );
        // SAFETY: The array is initialized.
        v.assume_init()
    }
}

const ELEMS_3: usize = 3;
const ELEMS_4: usize = 4;

#[derive(Copy, Clone, Debug)]
pub struct Translation(pub Translation3<f32>);

impl TestableFixedGattValue for Translation {
    const SIZE: usize = core::mem::size_of::<f32>() * ELEMS_3;

    fn from_gatt(data: &[u8]) -> Self {
        let data = vector_from_gatt::<ELEMS_3>(data);
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

#[derive(Copy, Clone, Debug)]
pub struct Vector(pub Vector3<f32>);

impl TestableFixedGattValue for Vector {
    const SIZE: usize = core::mem::size_of::<f32>() * ELEMS_3;

    fn from_gatt(data: &[u8]) -> Self {
        let data = vector_from_gatt::<ELEMS_3>(data);
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

#[derive(Copy, Clone, Debug)]
pub struct UQuaternion(pub UnitQuaternion<f32>);

impl TestableFixedGattValue for UQuaternion {
    const SIZE: usize = core::mem::size_of::<f32>() * ELEMS_4;

    fn from_gatt(data: &[u8]) -> Self {
        let data = vector_from_gatt::<ELEMS_4>(data);
        Self(UnitQuaternion::new_unchecked(Quaternion::from_vector(data)))
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

macro_rules! always_aligned_gatt_impl {
    ($ty:ty) => {
        impl TestableFixedGattValue for $ty {
            const SIZE: usize = core::mem::size_of::<Self>();

            fn from_gatt(data: &[u8]) -> Self {
                if data.len() != Self::SIZE {
                    panic!("Bad len")
                }
                unsafe { *(data.as_ptr() as *const Self) }
            }

            fn to_gatt(&self) -> &[u8] {
                unsafe { core::slice::from_raw_parts(self as *const Self as *const u8, Self::SIZE) }
            }
        }
    };
}
always_aligned_gatt_impl!(u8);
always_aligned_gatt_impl!(i8);

/// # Safety
/// data should be valid for reads and writes.
pub unsafe trait AlignedPrimitive: Copy {}
unsafe impl AlignedPrimitive for u16 {}
unsafe impl AlignedPrimitive for u32 {}
unsafe impl AlignedPrimitive for u64 {}
unsafe impl AlignedPrimitive for i16 {}
unsafe impl AlignedPrimitive for i32 {}
unsafe impl AlignedPrimitive for i64 {}
unsafe impl AlignedPrimitive for f32 {}
unsafe impl AlignedPrimitive for f64 {}

impl<T: AlignedPrimitive> TestableFixedGattValue for T {
    const SIZE: usize = core::mem::size_of::<f32>();

    fn from_gatt(data: &[u8]) -> Self {
        assert!(data.len() == Self::SIZE);

        // Safety: data should be valid for reads and contain properly
        // initialized values of T.
        unsafe { core::ptr::read_unaligned(data.as_ptr() as *const T) }
    }

    fn to_gatt(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(self as *const _ as *const u8, core::mem::size_of::<f32>())
        }
    }
}

impl TestableFixedGattValue for bool {
    const SIZE: usize = 1;

    fn from_gatt(data: &[u8]) -> Self {
        data != [0x00]
    }

    fn to_gatt(&self) -> &[u8] {
        match self {
            true => &[0x01],
            false => &[0x00],
        }
    }
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SM(pub StateMachine);

impl TestableFixedGattValue for SM {
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
