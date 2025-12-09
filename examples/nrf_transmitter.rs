#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::{debug, error, info, warn};
use embedded_nrf24l01::{Configuration, DataRate, NRF24L01};
use stm32f4xx_hal::{dwt::DwtExt, gpio::Speed, prelude::*, spi};
use panic_probe as _;
use defmt_rtt as _;

// --- Helper Macro for SPI Transfer ---
// Pulls CSN low, transfers data in place, and pulls CSN high.
macro_rules! nrf_transfer {
    ($spi:expr, $ncs:expr, $buf:expr) => {
        $ncs.set_low();
        match $spi.transfer_in_place($buf) {
            Ok(_) => {},
            Err(_) => error!("SPI Transfer Failed"),
        }
        $ncs.set_high();
    };
}

macro_rules! pulse_ce {
    ($ce:expr, $delay:expr) => {
        $ce.set_high();
        $delay.delay_us(20); // Must be > 10us
        $ce.set_low();
    };
}

#[entry]
fn main() -> ! {
    // --- Hardware Setup ---
    let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(48.MHz()).require_pll48clk().freeze();

    let dwt = cp.DWT.constrain(cp.DCB, &clocks);
    let mut d = dwt.delay(); 

    debug!("HCLK: {} MHz", clocks.hclk().raw() / 1_000_000);
    debug!("PCLK1: {} MHz", clocks.pclk1().raw() / 1_000_000);
    debug!("PCLK2: {} MHz", clocks.pclk2().raw() / 1_000_000);

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    // Configure Pins
    let pa7 = gpioa.pa7.into_alternate().speed(Speed::VeryHigh); // MOSI
    let pa6 = gpioa.pa6.into_alternate().speed(Speed::VeryHigh); // MISO
    let pa5 = gpioa.pa5.into_alternate().speed(Speed::VeryHigh); // SCK

    let mut ce = gpiob.pb10.into_push_pull_output().speed(Speed::VeryHigh);
    let mut ncs = gpiob.pb1.into_push_pull_output().speed(Speed::VeryHigh);

    // Initial State
    ncs.set_high();
    ce.set_low(); // CE Low = Standby Mode (Configuration allowed)

    let mode = spi::Mode {
        polarity: spi::Polarity::IdleLow,
        phase: spi::Phase::CaptureOnFirstTransition,
    };

    // Initialize SPI
    let nrf_spi = spi::Spi::new(dp.SPI1, (pa5, pa6, pa7), mode, 1.MHz(), &clocks);

    let res = NRF24L01::new(ce, ncs, nrf_spi);
    let mut nrf = match res {
        Ok(n) => n,
        Err(e) => {
            warn!("NRF24L01+ Initialization Failed");
            loop {}
        }
    };

    nrf.set_frequency(73).unwrap();




    // nrf.set_frequency(73).unwrap(); // Channel 76
    // nrf.set_auto_retransmit(15, 15).unwrap(); // 15 Retries, 4000us delay (Max safety)
    // nrf.set_rf(&DataRate::R1Mbps, 0).unwrap(); // 1Mbps, 0dBm
    
    // // Pipe Addresses (Must match Receiver)
    // // We set both TX and RX_P0 to the same address for Auto-Ack to work
    // nrf.set_tx_addr(&[0xE7, 0xE7, 0xE7, 0xE7, 0xE7]).unwrap();
    // nrf.set_rx_addr(0, &[0xE7, 0xE7, 0xE7, 0xE7, 0xE7]).unwrap();
    
    // // Important: Library sets payload size dynamically by default, 
    // // but for stability with basic setups, ensure pipes are enabled.
    // nrf.set_pipes_rx_enable(&[true, false, false, false, false, false]).unwrap();
    
    // // Flush to start clean
    // nrf.flush_tx().unwrap();
    // nrf.flush_rx().unwrap();

    // let f = nrf.get_frequency().unwrap();
    // info!("Current NRF24L01+ Frequency Channel: {}", f);

    // let mut tx = nrf.tx().unwrap();


    // loop {
    //     let payload = b"HELLO"; // 5 bytes

    //     // 5. Send Packet
    //     // .send() queues the packet. 
    //     // .wait_empty() blocks until success (ACK) or fail (MAX_RT).
    //     match tx.send(payload) {
    //         Ok(_) => {
    //             match tx.wait_empty() {
    //                 Ok(_) => info!("Packet Sent! (ACK Received)"),
    //                 Err(e) => {
    //                     warn!("Packet Failed (Max Retries)");
    //                     tx.flush_tx().unwrap(); // Clear the stuck packet
    //                 }
    //             }
    //         },
    //         Err(e) => warn!("Queue Full or Error"),
    //     }

    //     d.delay_ms(100);
    // }
    loop {}
}