use core::{clone, marker::PhantomData};

use defmt::debug;
use nalgebra::{SMatrix, SVector};
use stm32f4xx_hal::{gpio, otg_fs::{USB, UsbBus}, spi::Spi};
use embedded_hal::delay::DelayNs;


use crate::mpu_spi::MpuSpi;

pub struct Unconnected;
pub struct Connected;

pub const PACKET_SIZE: usize = 48;

pub enum CalibratorStateWrapper<SPI: stm32f4xx_hal::spi::Instance, const P: char, const N: u8> {
    Unconnected(Calibrator<Unconnected, SPI, P, N>),
    Connected(Calibrator<Connected, SPI, P, N>),
    AccelCalib(Calibrator<AccelCalib, SPI, P, N>),
    GyroCalib(Calibrator<GyroCalib, SPI, P, N>),
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



#[derive(Debug)]
#[repr(C)]
pub struct UsbPacket {
    pub data: [u8; PACKET_SIZE],
    pub len: usize,
}

pub struct Calibrator<STATE, SPI: stm32f4xx_hal::spi::Instance, const P: char, const N: u8> {
    _state: PhantomData<STATE>,
    mpu_spi: MpuSpi<Spi<SPI>, gpio::Pin<P, N, gpio::Output>>,
    accel_type: AccelCommand,
    d: stm32f4xx_hal::dwt::Delay,
    mean_px: Option<SVector<f32, 3>>,
    mean_nx: Option<SVector<f32, 3>>,
    mean_py: Option<SVector<f32, 3>>,
    mean_ny: Option<SVector<f32, 3>>,
    mean_pz: Option<SVector<f32, 3>>,
    mean_nz: Option<SVector<f32, 3>>,
    s_inv: Option<SMatrix<f32, 3, 3>>,    // inverse sensitivity matrix
    b: Option<SVector<f32, 3>>,             // bias vector
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
        self.s_inv = Some(s.try_inverse().unwrap());
        self.b = Some(SVector::<f32, 3>::new(b_x, b_y, b_z));
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
            s_inv: None,
            b: None,
        }
    }

    #[allow(unused_mut)]
    pub fn process(mut self, data: UsbPacket) -> (CalibratorStateWrapper<SPI, P, N>, CalibratorResponse) {
        debug!("Calibrator<Unconnected> received data: {:?}", &data.data[..data.len]);
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
        debug!("Calibrator<Connected> received data: {:?}", &data.data[..data.len]);
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
        debug!("Calibrator<AccelCalib> received data: {:?}", &data.data[..data.len]);

        let mut calibdata = [0u8; PACKET_SIZE]; // placeholder for actual calibration data processing
        let (mut mean_x, mut mean_y, mut mean_z) = self.mpu_spi.read_accel();
        let mut n = 1.0f32;

        for i in 0..1000 {
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
                self.mean_px = Some(SVector::<f32, 3>::new(mean_x, mean_y, mean_z));
                self.accel_type = AccelCommand::NX;
                (CalibratorStateWrapper::AccelCalib(self), CalibratorResponse::ACK)
            },
            AccelCommand::NX => {
                self.mean_nx = Some(SVector::<f32, 3>::new(mean_x, mean_y, mean_z));
                self.accel_type = AccelCommand::PY;
                (CalibratorStateWrapper::AccelCalib(self), CalibratorResponse::ACK)
            },
            AccelCommand::PY => {
                self.mean_py = Some(SVector::<f32, 3>::new(mean_x, mean_y, mean_z));
                self.accel_type = AccelCommand::NY;
                (CalibratorStateWrapper::AccelCalib(self), CalibratorResponse::ACK)
            },
            AccelCommand::NY => {
                self.mean_ny = Some(SVector::<f32, 3>::new(mean_x, mean_y, mean_z));
                self.accel_type = AccelCommand::PZ;
                (CalibratorStateWrapper::AccelCalib(self), CalibratorResponse::ACK)
            },
            AccelCommand::PZ => {
                self.mean_pz = Some(SVector::<f32, 3>::new(mean_x, mean_y, mean_z));
                self.accel_type = AccelCommand::NZ;
                (CalibratorStateWrapper::AccelCalib(self), CalibratorResponse::ACK)
            },
            AccelCommand::NZ => {
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
                (CalibratorStateWrapper::Connected(self.transition_to()), CalibratorResponse::Data(resp, 48))
            },
        }
    }
}