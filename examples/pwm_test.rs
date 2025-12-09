#![no_std]
#![no_main]

use core::{cell::RefCell, ops::Mul, sync::atomic::AtomicBool};

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use defmt::{debug, info, println, warn};
use nalgebra::{SMatrix, SVector};
use panic_probe as _;
use static_cell::{ConstStaticCell, StaticCell};
use stm32f4xx_hal::{ClearFlags, dwt::DwtExt, gpio::Speed, hal::spi, otg_fs::{USB, UsbBus}, pac::{NVIC, TIM2}, timer::{CounterHz, Event, Flag}}; 
use defmt_rtt as _; 
use stm32f4xx_hal::prelude::*;
use usb_device::{bus::UsbBusAllocator, device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid}};
use usbd_serial::SerialPort;
use stm32f4xx_hal::interrupt;
use usb_device::device::UsbDeviceState::{Configured, Addressed, Suspend};
use heapless::spsc::{Producer, Queue};


// FC Shared Resources


#[entry]
fn main() -> ! {
    let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(48.MHz()).require_pll48clk().freeze();

    let dwt = cp.DWT.constrain(cp.DCB, &clocks);

    debug!("HCLK: {} MHz", clocks.hclk().raw() / 1_000_000);
    debug!("PCLK1: {} MHz", clocks.pclk1().raw() / 1_000_000);
    debug!("PCLK2: {} MHz", clocks.pclk2().raw() / 1_000_000);

    let gpioa = dp.GPIOA.split();

    let (_pwm_mng,(ch1, ch2, ch3, ch4)) = dp.TIM5.pwm_hz(60.kHz(), &clocks);

    
    // 4 channel PWM driver for DC motor control
    let mut ch1 = ch1.with(gpioa.pa0);
    let mut ch2 = ch2.with(gpioa.pa1);
    let mut ch3 = ch3.with(gpioa.pa2);
    let mut ch4 = ch4.with(gpioa.pa3);

    let maxc1 = ch1.get_max_duty() as u32;
    let maxc2 = ch2.get_max_duty() as u32;
    let maxc3 = ch3.get_max_duty() as u32;
    let maxc4 = ch4.get_max_duty() as u32;

    debug!("Max duty cycles: ch1: {}, ch2: {}, ch3: {}, ch4: {}", maxc1, maxc2, maxc3, maxc4);

    ch1.set_duty(0);  // to set duty cycle to an arbitary value just multiply max_duty by desired percentage kiran
    ch2.set_duty(0);
    ch3.set_duty(0);
    ch4.set_duty((maxc4) as u16);

    let test = (maxc4) as u16;

    debug!("Test duty cycle value: {}", test);

    ch1.enable();
    ch2.enable();
    ch3.enable();
    ch4.enable();


    debug!("PWM channels configured");

    loop {};
}
