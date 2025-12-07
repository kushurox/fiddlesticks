use core::{clone, marker::PhantomData};

use defmt::{Format, debug, warn};
use nalgebra::{SMatrix, SVector};
use stm32f4xx_hal::{gpio, otg_fs::{USB, UsbBus}, pac::sdio::resp, spi::Spi};
use embedded_hal::delay::DelayNs;


use crate::mpu_spi::MpuSpi;

pub struct Unconnected;
pub struct Connected;

pub const PACKET_SIZE: usize = 64;

pub enum CalibratorStateWrapper<SPI: stm32f4xx_hal::spi::Instance, const P: char, const N: u8> {
    Unconnected(Calibrator<Unconnected, SPI, P, N>),
    Connected(Calibrator<Connected, SPI, P, N>),
    AccelCalib(Calibrator<AccelCalib, SPI, P, N>),
    GyroCalib(Calibrator<GyroCalib, SPI, P, N>),
}

impl<SPI, const P: char, const N: u8> Format for CalibratorStateWrapper<SPI, P, N> 
where 
    SPI: stm32f4xx_hal::spi::Instance,
{
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            CalibratorStateWrapper::Unconnected(_) => defmt::write!(fmt, "Unconnected"),
            CalibratorStateWrapper::Connected(_) => defmt::write!(fmt, "Connected"),
            CalibratorStateWrapper::AccelCalib(_) => defmt::write!(fmt, "AccelCalib"),
            CalibratorStateWrapper::GyroCalib(_) => defmt::write!(fmt, "GyroCalib"),
        }
    }
}

pub enum CalibratorResponse {
    ACK,
    NACK,
    Data([u8; PACKET_SIZE], usize),
    Disconnect
}

impl CalibratorResponse {
    pub const ACK_ID: u8 = 0x70;
    pub const NACK_ID: u8 = 0x71;
    pub const DATA_ID: u8 = 0x72;
    pub const DISCONNECT_ID: u8 = 0x73;

    pub fn get_id(&self) -> u8 {
        match self {
            CalibratorResponse::ACK => CalibratorResponse::ACK_ID,
            CalibratorResponse::NACK => CalibratorResponse::NACK_ID,
            CalibratorResponse::Data(_, _) => CalibratorResponse::DATA_ID,
            CalibratorResponse::Disconnect => CalibratorResponse::DISCONNECT_ID,
        }
    }
}


// 6-point accel calibration positions
pub enum AccelCommand {
    PX,
    NX,
    PY,
    NY,
    PZ,
    NZ,
}
pub struct AccelCalib;

// gyro calibration state
pub struct GyroCalib;

pub enum CalibCommand {
    ACCEL,
    GYRO,
    LOAD,      // Load calibration data from memory
}

impl Format for AccelCommand {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            AccelCommand::PX => defmt::write!(fmt, "PX"),
            AccelCommand::NX => defmt::write!(fmt, "NX"),
            AccelCommand::PY => defmt::write!(fmt, "PY"),
            AccelCommand::NY => defmt::write!(fmt, "NY"),
            AccelCommand::PZ => defmt::write!(fmt, "PZ"),
            AccelCommand::NZ => defmt::write!(fmt, "NZ"),
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct UsbPacket {
    pub data: [u8; PACKET_SIZE],
    pub len: usize,
}

pub struct Calibrator<STATE, SPI: stm32f4xx_hal::spi::Instance, const P: char, const N: u8> {
    _state: PhantomData<STATE>,
    pub mpu_spi: MpuSpi<Spi<SPI>, gpio::Pin<P, N, gpio::Output>>,
    accel_type: AccelCommand,
    d: stm32f4xx_hal::dwt::Delay,
    mean_px: Option<SVector<f32, 3>>,
    mean_nx: Option<SVector<f32, 3>>,
    mean_py: Option<SVector<f32, 3>>,
    mean_ny: Option<SVector<f32, 3>>,
    mean_pz: Option<SVector<f32, 3>>,
    mean_nz: Option<SVector<f32, 3>>,
    gyro_bias: Option<SVector<f32, 3>>,
    pub s_inv: Option<SMatrix<f32, 3, 3>>,    // inverse sensitivity matrix
    pub b: Option<SVector<f32, 3>>,             // bias vector
}

impl<STATE, SPI: stm32f4xx_hal::spi::Instance, const P: char, const N: u8> Calibrator<STATE, SPI, P, N> {
    fn transition_to<T>(self) -> Calibrator<T, SPI, P, N> {
        Calibrator {
            _state: PhantomData,
            mpu_spi: self.mpu_spi,
            accel_type: self.accel_type,
            d: self.d,
            mean_px: self.mean_px,
            mean_nx: self.mean_nx,
            mean_py: self.mean_py,
            mean_ny: self.mean_ny,
            mean_pz: self.mean_pz,
            mean_nz: self.mean_nz,
            gyro_bias: self.gyro_bias,
            s_inv: self.s_inv,
            b: self.b,
        }
    }

    fn compute_s_b(&mut self){
        // all mean values must be Some at this point. no checks will be performed since this can be called only after NZ which is the last step
        let mpx = self.mean_px.unwrap();
        let mnx = self.mean_nx.unwrap();
        let mpy = self.mean_py.unwrap();
        let mny = self.mean_ny.unwrap();
        let mpz = self.mean_pz.unwrap();
        let mnz = self.mean_nz.unwrap();

        let s_xx = (mpx.x - mnx.x) / 2.0;
        let s_xy = (mpy.x - mny.x) / 2.0;
        let s_xz = (mpz.x - mnz.x) / 2.0;
        let s_yx = (mpx.y - mnx.y) / 2.0;
        let s_yy = (mpy.y - mny.y) / 2.0;
        let s_yz = (mpz.y - mnz.y) / 2.0;

        let s_zx = (mpx.z - mnx.z) / 2.0;
        let s_zy = (mpy.z - mny.z) / 2.0;
        let s_zz = (mpz.z - mnz.z) / 2.0;

        let b_x = (mpx.x + mnx.x) / 2.0;
        let b_y = (mpy.y + mny.y) / 2.0;
        let b_z = (mpz.z + mnz.z) / 2.0;

        let s: SMatrix<f32, 3, 3> = SMatrix::<f32, 3, 3>::new(
            s_xx, s_xy, s_xz,
            s_yx, s_yy, s_yz,
            s_zx, s_zy, s_zz,
        );
        if let Some(s_inv) = s.try_inverse() {
            debug!("Computed sensitivity matrix S");
            debug!("Computed inverse sensitivity matrix S_inv");
            print_matrix(&s_inv);
            self.s_inv = Some(s_inv);
            self.b = Some(SVector::<f32, 3>::new(b_x, b_y, b_z));
        } else {
            debug!("Warning: Sensitivity matrix is non-invertible!");
        }
    }
}

impl<SPI: stm32f4xx_hal::spi::Instance, const P: char, const N: u8> Calibrator<Unconnected, SPI, P, N> {
    pub fn new(mpu_spi: MpuSpi<Spi<SPI>, gpio::Pin<P, N, gpio::Output>>, d: stm32f4xx_hal::dwt::Delay) -> Self {
        Calibrator {
            _state: PhantomData,
            mpu_spi,
            accel_type: AccelCommand::PX, // default accel calibration state
            d,
            mean_px: None,
            mean_nx: None,
            mean_py: None,
            mean_ny: None,
            mean_pz: None,
            mean_nz: None,
            gyro_bias: None,
            s_inv: None,
            b: None,
        }
    }

    #[allow(unused_mut)]
    pub fn process(mut self, data: UsbPacket) -> (CalibratorStateWrapper<SPI, P, N>, CalibratorResponse) {
        if data.data[..data.len].starts_with(b"CONNECT") {
            debug!("Calibrator START command received");
            (CalibratorStateWrapper::Connected(self.transition_to()), CalibratorResponse::ACK)
        } else {
            debug!("Calibrator<Unconnected> unknown command received");
            (CalibratorStateWrapper::Unconnected(self), CalibratorResponse::NACK)
        }
    }

}


impl<SPI: stm32f4xx_hal::spi::Instance, const P: char, const N: u8> Calibrator<Connected, SPI, P, N> {
    pub fn process(mut self, data: UsbPacket) -> (CalibratorStateWrapper<SPI, P, N>, CalibratorResponse) {
        if data.data.starts_with(b"ACCELPX") {
            debug!("Calibrator ACCEL PX command received");
            self.accel_type = AccelCommand::PX;
            (CalibratorStateWrapper::AccelCalib(self.transition_to()), CalibratorResponse::ACK)
        } else if data.data.starts_with(b"GYRO") {
            debug!("Calibrator GYRO command received");
            (CalibratorStateWrapper::GyroCalib(self.transition_to()), CalibratorResponse::ACK)
        } else if data.data.starts_with(b"DISCONNECT") {
            debug!("Calibrator DISCONNECT command received");
            (CalibratorStateWrapper::Unconnected(self.transition_to()), CalibratorResponse::Disconnect)
        } else {
            debug!("Calibrator<Connected> unknown command received");
            (CalibratorStateWrapper::Connected(self), CalibratorResponse::NACK)
        }
    }
}

impl<SPI, const P: char, const N: u8> Calibrator<AccelCalib, SPI, P, N>
where 
    SPI: stm32f4xx_hal::spi::Instance,

{
    pub fn process(mut self, data: UsbPacket) -> (CalibratorStateWrapper<SPI, P, N>, CalibratorResponse) {

        if data.data.starts_with(b"DISCONNECT") {
            warn!("Calibrator DISCONNECT command received during accel calibration, aborting calibration");
            return (CalibratorStateWrapper::Unconnected(self.transition_to()), CalibratorResponse::Disconnect);
        }

        let (mut mean_x, mut mean_y, mut mean_z) = self.mpu_spi.read_accel();
        let mut n = 1.0f32;
        debug!("Starting accel calibration for position {:?}", self.accel_type);

        for _i in 0..1000 {
            let x = self.mpu_spi.read_accel();
            n += 1.0;
            mean_x = mean_x + (x.0 - mean_x) / n;
            mean_y = mean_y + (x.1 - mean_y) / n;
            mean_z = mean_z + (x.2 - mean_z) / n;          
            self.d.delay_us(1500); // wait 1.5ms between samples
        }

        debug!("Accel calib mean values: X: {}, Y: {}, Z: {}", mean_x, mean_y, mean_z);
        match self.accel_type {
            AccelCommand::PX => {
                debug!("Calibrator Accel PX command performed");
                self.mean_px = Some(SVector::<f32, 3>::new(mean_x, mean_y, mean_z));
                self.accel_type = AccelCommand::NX;
                (CalibratorStateWrapper::AccelCalib(self), CalibratorResponse::ACK)
            },
            AccelCommand::NX => {
                debug!("Calibrator Accel NX command performed");
                self.mean_nx = Some(SVector::<f32, 3>::new(mean_x, mean_y, mean_z));
                self.accel_type = AccelCommand::PY;
                (CalibratorStateWrapper::AccelCalib(self), CalibratorResponse::ACK)
            },
            AccelCommand::PY => {
                debug!("Calibrator Accel PY command performed");
                self.mean_py = Some(SVector::<f32, 3>::new(mean_x, mean_y, mean_z));
                self.accel_type = AccelCommand::NY;
                (CalibratorStateWrapper::AccelCalib(self), CalibratorResponse::ACK)
            },
            AccelCommand::NY => {
                debug!("Calibrator Accel NY command performed");
                self.mean_ny = Some(SVector::<f32, 3>::new(mean_x, mean_y, mean_z));
                self.accel_type = AccelCommand::PZ;
                (CalibratorStateWrapper::AccelCalib(self), CalibratorResponse::ACK)
            },
            AccelCommand::PZ => {
                debug!("Calibrator Accel PZ command performed");
                self.mean_pz = Some(SVector::<f32, 3>::new(mean_x, mean_y, mean_z));
                self.accel_type = AccelCommand::NZ;
                (CalibratorStateWrapper::AccelCalib(self), CalibratorResponse::ACK)
            },
            AccelCommand::NZ => {
                debug!("Calibrator Accel NZ command performed");
                self.mean_nz = Some(SVector::<f32, 3>::new(mean_x, mean_y, mean_z));
                self.compute_s_b();
                let raw_data_s_inv: [u8; 36] = unsafe { core::mem::transmute(self.s_inv.clone().unwrap()) }; // nalgebra stores matrices in column-major order
                let raw_data_b: [u8; 12] = unsafe { core::mem::transmute(self.b.clone().unwrap()) };
                let resp: [u8; PACKET_SIZE] = {
                    let mut buf = [0u8; PACKET_SIZE];
                    buf[..36].copy_from_slice(&raw_data_s_inv);
                    buf[36..48].copy_from_slice(&raw_data_b);
                    buf
                };
                debug!("Accel calibration completed, sending S_inv and b to host");
                (CalibratorStateWrapper::Connected(self.transition_to()), CalibratorResponse::Data(resp, 48))
            },
        }
    }
}

impl<SPI, const P: char, const N: u8> Calibrator<GyroCalib, SPI, P, N>
where 
    SPI: stm32f4xx_hal::spi::Instance,

{
    pub fn process(mut self, _data: UsbPacket) -> (CalibratorStateWrapper<SPI, P, N>, CalibratorResponse) {
        // Gyro calibration not implemented yet

        let (gyro_x, gyro_y, gyro_z) = self.mpu_spi.calibrate_gyro();
        self.gyro_bias = Some(SVector::<f32, 3>::new(gyro_x, gyro_y, gyro_z));

        let resp : [u8; PACKET_SIZE] = {
            let mut buf = [0u8; PACKET_SIZE];
            let raw_data_gyro_bias: [u8; 12] = unsafe { core::mem::transmute(self.gyro_bias.clone().unwrap()) };
            buf[..12].copy_from_slice(&raw_data_gyro_bias);
            buf
        };

        debug!("Gyro calibration completed, sending bias to host");
        (CalibratorStateWrapper::Connected(self.transition_to()), CalibratorResponse::Data(resp, 12))
    }
}

fn print_matrix(s: &SMatrix<f32, 3, 3>) {
    debug!("Matrix:");
    for i in 0..3 {
        debug!("| {}, {}, {} |", s[(i,0)], s[(i,1)], s[(i,2)]);
    }
}