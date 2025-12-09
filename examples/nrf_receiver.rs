#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::{debug, error, info, warn};
use stm32f4xx_hal::{
    dwt::DwtExt,
    gpio::Speed,
    prelude::*,
    spi,
};
use embedded_nrf24l01::{Configuration, CrcMode, DataRate, NRF24L01};
use panic_probe as _;
use defmt_rtt as _;

#[entry]
fn main() -> ! {
    // --- Hardware Setup ---
    let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(48.MHz()).require_pll48clk().freeze();

    // DWT for delays
    let dwt = cp.DWT.constrain(cp.DCB, &clocks);
    let mut d = dwt.delay();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    // --- Configure Pins ---
    // MOSI: PA7, MISO: PA6, SCK: PA5
    let pa7 = gpioa.pa7.into_alternate().speed(Speed::VeryHigh); 
    let pa6 = gpioa.pa6.into_alternate().speed(Speed::VeryHigh); 
    let pa5 = gpioa.pa5.into_alternate().speed(Speed::VeryHigh); 

    // CE: PB10, CSN: PB1
    let ce = gpiob.pb10.into_push_pull_output().speed(Speed::VeryHigh);
    let csn = gpiob.pb1.into_push_pull_output().speed(Speed::VeryHigh);

    // --- SPI Setup ---
    let mode = spi::Mode {
        polarity: spi::Polarity::IdleLow,
        phase: spi::Phase::CaptureOnFirstTransition, // Mode 0
    };

    // 4 MHz is safe and stable
    let nrf_spi = spi::Spi::new(dp.SPI1, (pa5, pa6, pa7), mode, 4.MHz(), &clocks);

    // --- NRF24L01 Initialization ---
    let mut nrf = NRF24L01::new(ce, csn, nrf_spi).map_err(|_| {
        error!("Failed to initialize NRF24L01 structure");
    }).unwrap();

    // --- Critical Configuration ---
    // 1. Set Frequency to 76 (2476MHz) to match Arduino default
    nrf.set_frequency(76).unwrap();

    // 2. Set Data Rate 1Mbps, Power 0dBm
    nrf.set_rf(&DataRate::R1Mbps, 0).unwrap();

    // 3. Disable Auto-ACK (Must match Arduino)
    nrf.set_auto_ack(&[false; 6]).unwrap();

    // 4. Set Address "00001"
    let address = b"00001"; 
    nrf.set_rx_addr(0, address).unwrap();

    // 5. Payload Config
    // Arduino sends "Hello World" (11 chars + 1 null = 12 bytes)

    // 6. Enable Pipe 0
    nrf.set_pipes_rx_enable(&[true, false, false, false, false, false]).unwrap();
    nrf.set_pipes_rx_lengths(&[Some(5), None, None, None, None, None]).unwrap();

    // 7. CRC Config
    // Arduino RF24 library defaults to 16-bit CRC. We MUST match this.
    nrf.set_crc(CrcMode::TwoBytes).unwrap();

    // 8. Flush buffers
    nrf.flush_rx().unwrap();
    nrf.flush_tx().unwrap();

    info!("Config complete. Listening for 12 bytes on Channel {}...", nrf.get_frequency().unwrap());

    // Switch to RX Mode
    let mut rx = nrf.rx().unwrap();

    loop {
        // can_read() returns Result<Option<u8>, E>
        match rx.can_read() {
            Ok(Some(_pipe_no)) => {
                // Data is ready!
                match rx.read() {
                    Ok(packet) => {
                        let data = packet.as_ref();
                        let msg = core::str::from_utf8(data).unwrap_or("Invalid UTF-8");
                        info!("Received: {:?}", msg);
                    },
                    Err(_) => error!("FIFO Read Error"),
                }
            },
            Ok(None) => {
                // No data in FIFO
                d.delay_us(100); 
            },
            Err(_) => {
                // Hardware communication error (often means loose wires)
                error!("SPI Error - check wiring");
            }
        }
    }
}