#![no_std]
#![no_main]

use core::{cell::RefCell, sync::atomic::AtomicBool};

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use defmt::{debug, info, println, warn};
use nalgebra::{SMatrix, SVector, geometry::Quaternion};
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

use crate::{calib::{Calibrator, CalibratorResponse, CalibratorStateWrapper, PACKET_SIZE, UsbPacket}, mpu_spi::MpuSpi};

mod mpu_spi;
mod calib;
mod madgwick;

type SharedObj<T> = Mutex<RefCell<Option<T>>>;

// USB Shared Resources
static USB_BUS: StaticCell<UsbBusAllocator<UsbBus<USB>>> = StaticCell::new();
static USB_MEMORY: StaticCell<[u32; 1024]> = StaticCell::new(); // 1KB buffer for USB driver
static USB_SERIAL: SharedObj<SerialPort<UsbBus<USB>>> = SharedObj::new(RefCell::new(None));
static USB_DEVICE: SharedObj<UsbDevice<UsbBus<USB>>> = SharedObj::new(RefCell::new(None));
static USB_IS_CONNECTED: AtomicBool = AtomicBool::new(false);
static TIMER2: SharedObj<CounterHz<TIM2>> = SharedObj::new(RefCell::new(None));
static USB_QUEUE: ConstStaticCell<Queue<UsbPacket, 256>> = ConstStaticCell::new(Queue::new());
static USB_PRODUCER: SharedObj<Producer<UsbPacket>> = SharedObj::new(RefCell::new(None));

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

    let maxc1 = ch1.get_max_duty();
    let maxc2 = ch2.get_max_duty();
    let maxc3 = ch3.get_max_duty();
    let maxc4 = ch4.get_max_duty();

    ch1.set_duty((maxc1 * 80)/100);  // to set duty cycle to an arbitary value just multiply max_duty by desired percentage kiran
    ch2.set_duty((maxc2 * 70)/100);
    ch3.set_duty((maxc3 * 60)/100);
    ch4.set_duty((maxc4 * 50)/100);
    debug!("PWM channels configured");

    // USB initialization

    let usb_mem = USB_MEMORY.init([0; 1024]); // initialize the static USB memory

    let descriptors = StringDescriptors::default()
    .manufacturer("kushurox")
    .product("fiddlesticks")
    .serial_number("1");


    let usb = USB::new((dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK), (gpioa.pa11, gpioa.pa12), &clocks);
    let usb_bus = USB_BUS.init(UsbBus::new(usb, usb_mem));
    // note to future, call this before building UsbDevice, else HardFault occurs cause usebuilder freezes the bus
    let usb_serial_port = SerialPort::new(usb_bus); 

    let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
    .device_class(usbd_serial::USB_CLASS_CDC)
    .strings(&[descriptors])
    .unwrap()
    .build();

    let usb_queue = USB_QUEUE.take();
    let (usb_producer, mut usb_consumer) = usb_queue.split();


    let mut tim2 = dp.TIM2.counter_hz(&clocks);
    tim2.listen(Event::Update);

    cortex_m::interrupt::free(|cs| {
        unsafe { NVIC::unmask(interrupt::OTG_FS); }

        USB_DEVICE.borrow(cs).replace(Some(usb_dev));
        USB_SERIAL.borrow(cs).replace(Some(usb_serial_port));
        USB_PRODUCER.borrow(cs).replace(Some(usb_producer));

        TIMER2.borrow(cs).replace(Some(tim2));
    });

    debug!("USB device configured");


    // MPU9250 initialization can be done here

    let gpiob = dp.GPIOB.split();
    let mut pa8 = gpioa.pa8.into_push_pull_output().speed(Speed::High); // ncs
    let pb14 = gpiob.pb14.into_alternate().speed(Speed::VeryHigh); // AD0/MISO
    let pb15 = gpiob.pb15.into_alternate().speed(Speed::VeryHigh); // MOSI
    let pb13 = gpiob.pb13.into_alternate().speed(Speed::VeryHigh); // SCLK

    pa8.set_high();

    let mode = spi::Mode {
        polarity: spi::Polarity::IdleLow,
        phase: spi::Phase::CaptureOnFirstTransition,
    };

    let spi = stm32f4xx_hal::spi::Spi::new(dp.SPI2, (pb13, pb14, pb15), mode, 1.MHz(), &clocks);
    let ncs = pa8;


    // for my board X -> -X, Y -> -Y, Z -> Z
    let mut mpu_spi = MpuSpi::new(spi, ncs, dwt.delay());
    mpu_spi.get_status();
    let (gx, gy, gz) = mpu_spi.calibrate_gyro();
    info!("MPU6050 initialized and gyro calibrated. Gyro biases: x: {}, y: {}, z: {}", gx, gy, gz);

    let mut usb_calibrator = CalibratorStateWrapper::Unconnected(Calibrator::new(mpu_spi, dwt.delay()));
    const IS_CALIBRATED: bool = true;
    let mut calibrating = !IS_CALIBRATED; // runtime flag to break out of calibration loop

    let mut inv_sens_matrix: SMatrix::<f32, 3, 3>;
    let mut accel_bias: SVector::<f32, 3>;


    if IS_CALIBRATED {
        // hardcoded calibration values
        inv_sens_matrix = SMatrix::<f32, 3, 3>::from_column_slice(&[1.000258, 0.009667, -0.062812, -0.027891, 1.006843, -0.004916, 0.048179, -0.003334, 0.987162]);
        accel_bias = SVector::<f32, 3>::from_column_slice(&[0.014957, -0.006752, 0.302356]);
    } else {
        inv_sens_matrix = SMatrix::<f32, 3, 3>::zeros();
        accel_bias = SVector::<f32, 3>::zeros();
    }

    if !IS_CALIBRATED {
        while calibrating {
            if usb_consumer.is_empty() {
                continue;
            }
            use CalibratorStateWrapper::*;
            let packet = usb_consumer.dequeue().unwrap();
            let mut response_data: [u8; PACKET_SIZE] = [0u8; PACKET_SIZE];
            let response_id: u8;
            let response_length: usize;

            let recived_str = str::from_utf8(&packet.data[..packet.len]);
            if let Ok(s) = recived_str {
                info!("received packet: {}", s);
            } else {
                info!("received non-UTF8 packet");
            }

            usb_calibrator = match usb_calibrator {
                AccelCalib(calib) => {
                    let (calib, response) = calib.process(packet);
                    if let CalibratorResponse::Data(data, len) = response {
                        response_id = response.get_id();
                        response_length = len;
                        response_data[..len].copy_from_slice(&data[..len]);
                    } else {
                        response_id = response.get_id();
                        response_length = 1;
                    }
                    calib
                },
                GyroCalib(calib) => {
                    let (calib, response) = calib.process(packet);
                    if let CalibratorResponse::Data(data, len) = response {
                        response_id = response.get_id();
                        response_length = len;
                        response_data[..len].copy_from_slice(&data[..len]);
                    } else {
                        response_id = response.get_id();
                        response_length = 1;
                    }
                    calib
                },
                Unconnected(calib) => {
                    let (calib, response) = calib.process(packet);
                    response_id = response.get_id(); // cause only ACK/NACK expected
                    response_length = 1;
                    calib
                },
                Connected(calib) => {
                    debug!("In Connected state");
                    let (calibstate, response) = calib.process(packet);
                    match response {
                        CalibratorResponse::Disconnect => {
                            debug!("Disconnect received, exiting calibration loop");
                            calibrating = false;

                            if let Unconnected(ref inner_calib) = calibstate {
                                if let Some(s_inv) = &inner_calib.s_inv {
                                    inv_sens_matrix = *s_inv;
                                    info!("Copied inverse sensitivity matrix");
                                } else { warn!("No sensitivity matrix found!"); }
                                if let Some(bias) = &inner_calib.b {
                                    accel_bias = *bias;
                                    info!("Copied accelerometer bias vector");
                                } else { warn!("No accelerometer bias vector found!"); }
                            }
                        },
                        _ => {}

                    };
                    response_id = response.get_id(); // cause only ACK/NACK expected
                    response_length = 1;
                    calibstate
                }
            };

            println!("Response packet length: {}", response_length);
            const USIZE_SIZE: usize = core::mem::size_of::<usize>();
            let mut usb_raw_packet: [u8; 1+USIZE_SIZE+PACKET_SIZE] = [0u8; 1+USIZE_SIZE+PACKET_SIZE];
            usb_raw_packet[0] = response_id; // response ID
            usb_raw_packet[1..=USIZE_SIZE].copy_from_slice(&response_length.to_le_bytes());
            usb_raw_packet[1+USIZE_SIZE..1+USIZE_SIZE+response_length].copy_from_slice(&response_data[..response_length]);

            cortex_m::interrupt::free(|cs| {
                let res = USB_SERIAL.borrow(cs).borrow_mut().as_mut().map(|serial| {
                    if let Err(_) = serial.write(&usb_raw_packet) {
                        warn!("USB write error");
                        panic!("USB write error"); // ill spin it later, for now just panic
                    }
                });
                if let None = res {
                    warn!("USB serial port not available");
                    panic!("USB serial port not available"); 
                }
            });

        };

    }

    // release mpu_spi from calibrator
    let mut mpu_spi = match usb_calibrator {
        CalibratorStateWrapper::Unconnected(calib) => calib.mpu_spi,
        CalibratorStateWrapper::Connected(calib) => calib.mpu_spi,
        _ => panic!("Calibrator in invalid state after calibration"),
    };

    let mut madgwick_filter = madgwick::MadgwickFilter::new(0.1); // beta value can be tuned

    loop {
        let dt = 0.0025f32; // for 400Hz update rate
        let (acc_x, acc_y, acc_z) = mpu_spi.read_accel();
        let (gyro_x, gyro_y, gyro_z) = mpu_spi.read_gyro();

        let acc_vector = SVector::<f32, 3>::from_column_slice(&[acc_x, acc_y, acc_z]);
        let gyro_vector = SVector::<f32, 3>::from_column_slice(&[-gyro_x.to_radians(), -gyro_y.to_radians(), gyro_z.to_radians()]);

        let mut calibrated_acc = inv_sens_matrix * (acc_vector - accel_bias);
        calibrated_acc.x = -calibrated_acc.x;
        calibrated_acc.y = -calibrated_acc.y;

        madgwick_filter.update_imu(gyro_vector, calibrated_acc, dt);

        let angles = madgwick_filter.to_euler_angles();
        let roll = angles.x.to_degrees();
        let pitch = angles.y.to_degrees();
        let yaw = angles.z.to_degrees();

        println!("Roll: {}, Pitch: {}, Yaw: {}", roll, pitch, yaw);

        dwt.delay().delay_us(2500);
    }
}

#[interrupt]
fn OTG_FS() {
    let (usb_dev, usb_serial) = cortex_m::interrupt::free(|cs| {
        (
            USB_DEVICE.borrow(cs).borrow_mut().take(),
            USB_SERIAL.borrow(cs).borrow_mut().take()
        )
    });

    let (mut usb_dev, mut usb_serial) = match (usb_dev, usb_serial) {
        (Some(usb_dev), Some(usb_serial)) => (usb_dev, usb_serial),
        _ => return,
    };



    if usb_dev.poll(&mut [&mut usb_serial]) {
        let ustate = usb_dev.state();
        if let Configured = ustate {
            let mut packet = UsbPacket { data: [0u8; PACKET_SIZE], len:0 };
            if let Ok(count) = usb_serial.read(&mut packet.data) {
                if count > 0 { 
                    packet.len = count;
                    cortex_m::interrupt::free(|cs| {
                        let res = USB_PRODUCER.borrow(cs).borrow_mut().as_mut().map(|queue| {
                            queue.enqueue(packet).unwrap(); // panic if queue full
                        });
                        if let None = res {
                            warn!("USB producer queue not available");
                            panic!("USB producer queue not available");
                        }
                    });
                }
            }
        } else if let Addressed = ustate {
            use core::sync::atomic::Ordering::{Relaxed, Acquire};
            if let Ok(_) = USB_IS_CONNECTED.compare_exchange(false, true, Acquire, Relaxed) {
                // start timer2 interrupt to monitor connection
                cortex_m::interrupt::free(|cs| {
                    if let Some(tim2) = TIMER2.borrow(cs).borrow_mut().as_mut() {
                        tim2.start(100.Hz()).ok();
                        unsafe { NVIC::unmask(interrupt::TIM2); } // enable timer interrupts
                    }
                });
                info!("USB connected");
            }
        }
    }


    cortex_m::interrupt::free(|cs| {
        USB_DEVICE.borrow(cs).replace(Some(usb_dev));
        USB_SERIAL.borrow(cs).replace(Some(usb_serial));
    });

}

#[interrupt]
fn TIM2() {
    use core::sync::atomic::Ordering::Relaxed;
    
    let connected = USB_IS_CONNECTED.load(Relaxed);

    cortex_m::interrupt::free(|cs| {
        if let Some(tim2) = TIMER2.borrow(cs).borrow_mut().as_mut() {

            USB_DEVICE.borrow(cs).borrow().as_ref().map(|usb_dev| {
                if let Suspend = usb_dev.state() {
                    if connected {
                        info!("USB disconnected");
                        USB_IS_CONNECTED.store(false, Relaxed);
                        tim2.cancel().ok();
                        NVIC::mask(interrupt::TIM2); // stop timer interrupts
                    }
                }
            });
            tim2.clear_flags(Flag::Update);
        }
    });
}
