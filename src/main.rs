#![no_main]
#![no_std]

extern crate panic_halt;

use cortex_m_rt::entry;

#[entry]
fn entry() -> ! {
    loop {}
}
