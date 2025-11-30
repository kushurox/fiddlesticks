// just to store code for reference
// note the macro panics if spi fails since it unwraps the result
// select_write!(spi, ncs, &mut [0x6B, 0x80]); // wake up device
// dwt.delay().delay_ms(100);
// select_write!(spi, ncs, &mut [0x6B, 0x01]); // set clock source to gyro
// dwt.delay().delay_ms(200); // wait for PLL to stabilize

// select_write!(spi, ncs, &mut [0x6A, 0x10]); // disables I2C bus
// dwt.delay().delay_ms(10);
// select_write!(spi, ncs, &mut [0x1A, 0x01]); // setting DLPF to 184Hz
// dwt.delay().delay_ms(10);
// select_write!(spi, ncs, &mut [0x1B, 0x18]); // setting gyro full scale to 2000dps
// dwt.delay().delay_ms(10);
// select_write!(spi, ncs, &mut [0x1C, 0x18]); // setting accelerometer full scale to 16g
// dwt.delay().delay_ms(10);
// select_write!(spi, ncs, &mut [0x1D, 0x02]); // setting accelerometer DLPF to 99Hz
// dwt.delay().delay_ms(10);
// select_write!(spi, ncs, &mut [0x19, 0x00]); // setting sample rate to 1kHz
// dwt.delay().delay_ms(10);

    // let mut sensor_readings = [0u8; 15]; // first byte is register address, next 14 bytes are data
// sensor_readings[0] = 0x3B | 0x80;
// ncs.set_low();
// spi.transfer_in_place(&mut sensor_readings).unwrap();
// ncs.set_high();

// let acc_x = convert_raw(sensor_readings[1], sensor_readings[2], 2048.0);
// let gyro_x = convert_raw(sensor_readings[9], sensor_readings[10], 16.4);

// info!("Acc X: {} g, Gyro X: {} dps", acc_x, gyro_x);
// dwt.delay().delay_ms(500);