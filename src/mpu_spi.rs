use defmt::debug;
use stm32f4xx_hal::{gpio::{Output, Pin}, spi::{Instance, Spi}};
use embedded_hal::delay::DelayNs;


#[macro_export]
macro_rules! select_write {
    ($sp:expr,$ncs:expr,$data:expr) => {
        $ncs.set_low();
        $sp.write($data).unwrap();
        $ncs.set_high();
    };
}

#[macro_export]
macro_rules! select_read {
    ($sp:expr, $ncs:expr, $addr:expr) => {
        {
            let mut buf = [$addr | 0x80, 0x00];
            $ncs.set_low();
            $sp.transfer_in_place(&mut buf).unwrap();
            $ncs.set_high();
            buf[1]
        }
    };
}



#[inline(always)]
pub fn convert_raw(h: u8, l: u8, sf: f32) -> f32 {
    // high byte, low byte, scale factor

    let raw = (((h as u16) << 8) | (l as u16)) as i16;
    let rawf32 = raw as f32;
    rawf32 / sf

}

// I'll refactor the struct to house the sensor specific information in generics when I have time
// assumes 2000dps and 16g full scale ranges
pub struct MpuSpi<Spi, CS> {
    pub spi: Spi,
    pub ncs: CS,
    pub d: stm32f4xx_hal::dwt::Delay,
}


impl<SPI, const P: char, const N: u8> MpuSpi<Spi<SPI>, Pin<P, N, Output>>
where 
    SPI: Instance,
{
    pub fn new(mut spi: Spi<SPI>, mut ncs: Pin<P, N, Output>, mut d: stm32f4xx_hal::dwt::Delay) -> Self {
        // init the MPU6050 before returning the struct
        select_write!(spi, ncs, &mut [0x6B, 0x80]); // wake up device
        d.delay_ms(100);
        select_write!(spi, ncs, &mut [0x6B, 0x01]); // set clock source to gyro
        d.delay_ms(200); // wait for PLL to stabilize

        select_write!(spi, ncs, &mut [0x6A, 0x10]); // disables I2C bus
        d.delay_ms(10);
        select_write!(spi, ncs, &mut [0x1A, 0x01]); // setting DLPF to 184Hz
        d.delay_ms(10);
        select_write!(spi, ncs, &mut [0x1B, 0x18]); // setting gyro full scale to 2000dps
        d.delay_ms(10);
        select_write!(spi, ncs, &mut [0x1C, 0x18]); // setting accelerometer full scale to 16g
        d.delay_ms(10);
        select_write!(spi, ncs, &mut [0x1D, 0x02]); // setting accelerometer DLPF to 99Hz
        d.delay_ms(10);
        select_write!(spi, ncs, &mut [0x19, 0x00]); // setting sample rate to 1kHz
        d.delay_ms(10);
        MpuSpi { spi, ncs, d }
    }

    pub fn write(&mut self, data: &mut [u8]) {
        select_write!(self.spi, self.ncs, data);
    }

    pub fn read(&mut self, addr: u8) -> u8 {
        select_read!(self.spi, self.ncs, addr)
    }

    pub fn get_status(&mut self) {
        // prints all relevant status registers for debugging
        debug!("WHO_AM_I: 0x{:X}", self.read(0x75));
        debug!("PWR_MGMT_1: 0x{:X}", self.read(0x6B));
        debug!("SMPLRT_DIV: 0x{:X}", self.read(0x19));
        debug!("CONFIG: 0x{:X}", self.read(0x1A));
        debug!("GYRO_CONFIG: 0x{:X}", self.read(0x1B));
        debug!("ACCEL_CONFIG: 0x{:X}", self.read(0x1C));
        debug!("ACCEL_CONFIG2: 0x{:X}", self.read(0x1D));
    }

    pub fn read_sensors(&mut self) -> (f32, f32, f32, f32, f32, f32, f32) {
        // reads all sensors and returns a tuple of them, performs conversion internally
        let mut sensor_readings = [0u8; 15]; // first byte is
        sensor_readings[0] = 0x3B | 0x80;
        self.ncs.set_low();
        self.spi.transfer_in_place(&mut sensor_readings).unwrap();
        self.ncs.set_high();
        let acc_x = convert_raw(sensor_readings[1], sensor_readings[2], 2048.0);
        let acc_y = convert_raw(sensor_readings[3], sensor_readings[4], 2048.0);
        let acc_z = convert_raw(sensor_readings[5], sensor_readings[6], 2048.0);
        let temp = convert_raw(sensor_readings[7], sensor_readings[8], 333.87) + 21.0;
        let gyro_x = convert_raw(sensor_readings[9], sensor_readings[10], 16.4);
        let gyro_y = convert_raw(sensor_readings[11], sensor_readings[12], 16.4);
        let gyro_z = convert_raw(sensor_readings[13], sensor_readings[14], 16.4);

        (acc_x, acc_y, acc_z, temp, gyro_x, gyro_y, gyro_z)
    }
}
