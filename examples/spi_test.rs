#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::{debug, info};
use stm32f4xx_hal::{dwt::DwtExt, gpio::Speed, prelude::*, spi};
use panic_probe as _;
use defmt_rtt as _;

use fiddlesticks::mpu_spi::MpuSpi;


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
    let gpiob = dp.GPIOB.split();


    let mut pa8 = gpioa.pa8.into_push_pull_output().speed(Speed::High); // ncs
    let pb14 = gpiob.pb14.into_alternate().speed(Speed::VeryHigh); // AD0/MISO
    let pb15 = gpiob.pb15.into_alternate().speed(Speed::VeryHigh); // MOSI
    let pb13 = gpiob.pb13.into_alternate().speed(Speed::VeryHigh); // SCLK
    let mut ncs = pa8;

    ncs.set_high();

    let mode = spi::Mode {
        polarity: spi::Polarity::IdleLow,
        phase: spi::Phase::CaptureOnFirstTransition,
    };

    let m_spi = spi::Spi::new(dp.SPI2, (pb13, pb14, pb15), mode, 1.MHz(), &clocks);

    let mut mpu_spi = MpuSpi::new(m_spi, ncs, dwt.delay());
    mpu_spi.get_status();
    mpu_spi.calibrate_gyro();
    
    loop {
        let (accx, accy, accz) = mpu_spi.read_accel();
        let (gyrox, gyroy, gyroz) = mpu_spi.read_gyro_remapped();

        info!("ACC X: {}, Y: {}, Z: {}", accx, accy, accz);
        // info!("GYRO X: {}, Y: {}, Z: {}", gyrox, gyroy, gyroz);
        dwt.delay().delay_ms(500);

    }
}