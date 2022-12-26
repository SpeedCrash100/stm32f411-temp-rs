use core::ops::{AddAssign, Div, SubAssign};
use num_traits::FromPrimitive;

use heapless::Vec;

#[derive(Default)]
pub struct MovingAverage<T, const N: usize> {
    values: Vec<T, N>,
    acc: T,
    position: usize,
}

impl<T, const N: usize> MovingAverage<T, N>
where
    T: AddAssign + SubAssign + Div<Output = T> + Copy + FromPrimitive,
{
    pub fn filter(&mut self, value: T) -> T {
        if let Some(val) = self.values.get_mut(self.position) {
            self.acc -= *val;
            *val = value;
        } else {
            self.values.push(value).ok();
        }

        self.acc += value;
        self.position += 1;
        self.position %= N;

        let d: T = T::from_usize(self.values.len()).expect("Cannot convert value");

        self.acc / d
    }
}
