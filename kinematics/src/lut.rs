use crate::ExpensiveMath;

// Skip this huge file
#[rustfmt::skip]
mod tables;

#[derive(Copy, Clone)]
pub struct LutMath;
const LUT_MUL: f32 = 2048.0 / core::f32::consts::PI;

impl ExpensiveMath<f32> for LutMath {
    fn atan2(l: f32, r: f32) -> f32 {
        let y = (l + 256.0).clamp(0.0, 511.0);
        let x = (r + 256.0).clamp(0.0, 511.0);
        tables::ATAN2[y as usize][x as usize]
        // let map = tables::ATAN2[y as usize][x as usize];
        // let left_y = y - 0.5;
        // let right_y = y + 0.5;
        // let left_x = x - 0.5;
        // let right_x = x + 0.5;
        // let ll = tables::ATAN2[left_y as usize][left_x as usize];
        // let lr = tables::ATAN2[left_y as usize][right_x as usize];
        // let rr = tables::ATAN2[right_y as usize][right_x as usize];
        // let rl = tables::ATAN2[right_y as usize][left_x as usize];
        // let mapped_left_x = ll * (left_y - y).abs() + rl * (right_y - y).abs();
        // let mapped_right_x = lr * (left_y - y).abs() + rr * (right_y - y).abs();
        // let lerp = mapped_left_x * (left_x - x).abs() + mapped_right_x * (right_x - x).abs();
        // println!("atan2({l},{r}) = {map} vs {lerp} vs {}", l.atan2(r));
        // lerp
    }

    #[inline(always)]
    fn acos(f: f32) -> f32 {
        let idx = (((f + core::f32::consts::PI) * LUT_MUL) as usize).clamp(0, 4096);
        tables::ACOS[idx]
    }

    #[inline(always)]
    fn sin(f: f32) -> f32 {
        let idx = (((f + core::f32::consts::PI) * LUT_MUL) as usize).clamp(0, 4095);
        tables::SIN[idx]
        // float_funcs::fsin(f)
    }

    #[inline(always)]
    fn cos(f: f32) -> f32 {
        let idx = (((f + core::f32::consts::PI) * LUT_MUL) as usize).clamp(0, 4095);
        tables::COS[idx]
        // float_funcs::fcos(f)
    }

    #[inline(always)]
    fn sincos(f: f32) -> (f32, f32) {
        let idx = (((f + core::f32::consts::PI) * LUT_MUL) as usize).clamp(0, 4095);
        (tables::SIN[idx], tables::COS[idx])
    }

    #[inline(always)]
    fn sqrt(f: f32) -> f32 {
        // let mut tmp: i32 = f.to_bits() as i32;
        // tmp -= 127 << 23; // Remove IEEE bias from exponent (-2^23)
        // // tmp is now an appoximation to logbase2(val)
        // tmp >>= 1; // divide by 2
        // tmp += 127 >> 23; // restore the IEEE bias from the exponent (+2^23)
        // let res = f32::from_bits(tmp as u32);
        let mut y = f;
        let mut i: u32 = y.to_bits();
        i = 0x5F375A86_u32.wrapping_sub(i >> 1);
        y = f32::from_bits(i);
        1.0 / (y * (1.5 - (f * 0.5 * y * y)))
    }
}
