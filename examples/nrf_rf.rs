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
use panic_probe as _;
use defmt_rtt as _;

// --- Register Map ---
const CONFIG: u8 = 0x00;
const EN_AA: u8 = 0x01;
const EN_RXADDR: u8 = 0x02;
const SETUP_AW: u8 = 0x03;
const RF_CH: u8 = 0x05;
const RF_SETUP: u8 = 0x06;
const STATUS: u8 = 0x07;
const RX_ADDR_P0: u8 = 0x0A;
const RX_PW_P0: u8 = 0x11;

// --- Commands ---
const R_REGISTER: u8 = 0x00;
const W_REGISTER: u8 = 0x20;
const R_RX_PAYLOAD: u8 = 0x61;
const FLUSH_TX: u8 = 0xE1;
const FLUSH_RX: u8 = 0xE2;

#[entry]
fn main() -> ! {
    let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(48.MHz()).require_pll48clk().freeze();
    
    let dwt = cp.DWT.constrain(cp.DCB, &clocks);
    let mut delay = dwt.delay();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    // --- Pin Config ---
    let pa5 = gpioa.pa5.into_alternate().speed(Speed::VeryHigh); // SCK
    let pa6 = gpioa.pa6.into_alternate().speed(Speed::VeryHigh); // MISO
    let pa7 = gpioa.pa7.into_alternate().speed(Speed::VeryHigh); // MOSI
    let mut ce = gpiob.pb10.into_push_pull_output().speed(Speed::VeryHigh);
    let mut csn = gpiob.pb1.into_push_pull_output().speed(Speed::VeryHigh);

    ce.set_low();
    csn.set_high();

    let mode = spi::Mode {
        polarity: spi::Polarity::IdleLow,
        phase: spi::Phase::CaptureOnFirstTransition,
    };
    
    let mut spi = spi::Spi::new(dp.SPI1, (pa5, pa6, pa7), mode, 4.MHz(), &clocks);
    delay.delay_ms(100); 

    // --- MACROS ---
    macro_rules! write_reg {
        ($reg:expr, $val:expr) => {
            {
                csn.set_low();
                let mut buf = [W_REGISTER | ($reg & 0x1F), $val];
                spi.transfer_in_place(&mut buf).unwrap(); 
                csn.set_high();
                delay.delay_us(10);
            }
        };
    }

    macro_rules! read_reg {
        ($reg:expr) => {
            {
                csn.set_low();
                let mut buf = [R_REGISTER | ($reg & 0x1F), 0x00];
                spi.transfer_in_place(&mut buf).unwrap();
                csn.set_high();
                delay.delay_us(10);
                buf[1]
            }
        };
    }

    macro_rules! send_cmd {
        ($cmd:expr) => {
            {
                csn.set_low();
                let mut buf = [$cmd];
                spi.transfer_in_place(&mut buf).unwrap();
                csn.set_high();
                delay.delay_us(10);
            }
        };
    }

    info!("Initializing Receiver (32 Byte Payload / 4 Floats)...");

    // 1. DISABLE Auto-ACK
    write_reg!(EN_AA, 0x00);
    // 2. Enable Pipe 0
    write_reg!(EN_RXADDR, 0x01);
    // 3. Address Width 5 bytes
    write_reg!(SETUP_AW, 0x03);
    // 4. RF Channel 76
    write_reg!(RF_CH, 76);
    // 5. 1Mbps, 0dBm
    write_reg!(RF_SETUP, 0x06);

    // 6. Set Address
    {
        csn.set_low();
        let mut addr_buf = [W_REGISTER | RX_ADDR_P0, 0x30, 0x30, 0x30, 0x30, 0x31];
        spi.transfer_in_place(&mut addr_buf).unwrap();
        csn.set_high();
    }

    // 7. PAYLOAD WIDTH: 32 BYTES (Matches Arduino struct)
    write_reg!(RX_PW_P0, 32);

    // 8. Config: RX Mode, Power Up, 16-bit CRC
    write_reg!(CONFIG, 0x0F);

    send_cmd!(FLUSH_RX);
    send_cmd!(FLUSH_TX);
    write_reg!(STATUS, 0x70); 

    // Verify
    let check_pw = read_reg!(RX_PW_P0);
    info!("Verify Payload Width: {} (Expected 32)", check_pw);
    
    if check_pw != 32 {
        error!("Payload width mismatch! Check wiring/power.");
    }

    ce.set_high(); 

    loop {
        let status = read_reg!(STATUS);
        
        if (status & 0x40) != 0 {
            csn.set_low();
            
            // Buffer: 1 command + 32 data bytes = 33 total
            // We read the full 32 bytes even though we only use the first 16.
            let mut rx_buf = [0u8; 33]; 
            rx_buf[0] = R_RX_PAYLOAD; 
            
            spi.transfer_in_place(&mut rx_buf).unwrap();
            csn.set_high();

            // The data is in bytes 1..33
            let raw = &rx_buf[1..33];

            // --- EXTRACT FLOATS ---
            // The Arduino struct is:
            // float j1x (0-4)
            // float j1y (4-8)
            // float j2x (8-12)
            // float j2y (12-16)
            // padding (16-32) -> Ignored

            let j1x = f32::from_le_bytes(raw[0..4].try_into().unwrap());
            let j1y = f32::from_le_bytes(raw[4..8].try_into().unwrap());
            let j2x = f32::from_le_bytes(raw[8..12].try_into().unwrap());
            let j2y = f32::from_le_bytes(raw[12..16].try_into().unwrap());

            // Print formatted output for the drone controller
            info!("Joy1: ({}, {})  Joy2: ({}, {})", j1x, j1y, j2x, j2y);

            write_reg!(STATUS, 0x40);
        }
        
        delay.delay_ms(2);
    }
}