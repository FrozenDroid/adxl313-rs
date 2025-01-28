#![no_std]

mod conf;
mod register;

use core::fmt::Debug;

use embedded_hal::{self as hal, i2c};

use hal::digital::OutputPin;
use hal::spi;

pub use accelerometer::{
    error,
    vector::{F32x3, I32x3},
    Accelerometer, Error, RawAccelerometer,
};

use crate::Adxl313Error::WrongId;
use crate::OutputDataRate::{ODR_1600_HZ, ODR_3200_HZ};
pub use conf::*;
use register::{bitmask, Register};

const SPI_READ: u8 = 0x80;
const SPI_WRITE: u8 = 0x00;

const EXPECTED_DEVID_0: u8 = 0xad;
const EXPECTED_DEVID_1: u8 = 0x1d;
const EXPECTED_PART_ID: u8 = 0b1100_1011;
const EXPECTED_DEVICE_ID: u32 = ((EXPECTED_DEVID_0 as u32) << 16)
    | ((EXPECTED_DEVID_1 as u32) << 8)
    | (EXPECTED_PART_ID as u32);

const ACCEL_MAX_16BIT: u32 = 32_767; // = 2^(16-1)-1
const ACCEL_MAX_10BIT: u32 = 511; // = 2^(10-1)-1

pub struct Spi<SPI: spi::SpiDevice, CS: OutputPin> {
    spi: SPI,
    cs: CS,
}

pub struct I2c<I2C> {
    pub i2c: I2C,
    pub address: u8,
}

pub trait Interface {
    type Error;
    fn write(&mut self, data: &mut [u8]) -> Result<(), Self::Error>;
    fn read(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error>;

    fn transfer(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error>;
}

impl<E> dyn Interface<Error = E> {
    pub fn new_spi<SPI: spi::SpiDevice, CS: OutputPin>(spi: SPI, cs: CS) -> Spi<SPI, CS> {
        Spi { spi, cs }
    }

    pub fn new_i2c<I2C: i2c::I2c>(i2c: I2C, address: u8) -> I2c<I2C> {
        I2c { i2c, address }
    }
}

impl<SPI: spi::SpiDevice, P: OutputPin> Interface for Spi<SPI, P> {
    type Error = SPI::Error;

    fn write(&mut self, data: &mut [u8]) -> Result<(), Self::Error> {
        data[0] |= SPI_READ;
        self.cs.set_low().ok();
        self.spi.write(data)?;
        self.cs.set_high().ok();
        Ok(())
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.cs.set_low().ok();
        self.spi.read(buffer)?;
        self.cs.set_high().ok();
        Ok(())
    }

    fn transfer(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.cs.set_low().ok();
        self.spi.transfer(read, write)?;
        self.cs.set_high().ok();
        Ok(())
    }
}

impl<I2C: i2c::I2c> Interface for I2c<I2C> {
    type Error = I2C::Error;

    fn write(&mut self, data: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write(self.address, data)
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.read(self.address, buffer)
    }

    fn transfer(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, write, read)
    }
}

/// ADXL313 driver
pub struct Adxl313<IF> {
    interface: IF,
    range: Option<Range>,
    left_justified_data: bool,
    full_resolution: bool,
    output_data_rate: Option<OutputDataRate>,
}

/// Errors in this crate
#[derive(Debug, PartialEq, Clone)]
pub enum Adxl313Error<InterfaceError> {
    InterfaceError(InterfaceError),
    WrongId(u32),
}

impl<IF, InterfaceError> Adxl313<IF>
where
    IF: Interface<Error = InterfaceError>,
{
    /// Create a new Adxl313 instance using the defaults
    pub fn new(interface: IF) -> Result<Self, Adxl313Error<InterfaceError>> {
        let mut adxl313 = Adxl313 {
            interface,
            range: None,
            output_data_rate: None,
            left_justified_data: false,
            full_resolution: false,
        };

        adxl313.power_control(
            false,
            false,
            false,
            false,
            false,
            SleepModeFrequencyReadings::_1_HZ,
        )?;
        adxl313.data_format(
            false,
            SpiMode::_4_WIRE,
            IrqMode::ACTIVE_LOW,
            false,
            false,
            Range::_0d5G,
        )?;

        let id = adxl313.get_device_id()?;

        if id != EXPECTED_DEVICE_ID {
            Err(WrongId(id))
        } else {
            Ok(adxl313)
        }
    }

    /// The XID register stores a semiunique serial number that is generated from the device trim
    /// and calibration process
    pub fn xid(&mut self) -> Result<u8, Adxl313Error<InterfaceError>> {
        self.read_reg(Register::XID.addr())
    }

    /// Writing a value of 0x52 to Register 0x18 triggers the soft reset function of the ADXL313.
    /// The soft reset returns the ADXL313 to the beginning of its power-on initialization routine,
    /// clearing the configuration settings that were written to the memory map, which allows easy
    /// reconfiguration of the ADXL313 device.
    pub fn soft_reset(&mut self) -> Result<(), Adxl313Error<InterfaceError>> {
        let val = 0x52;
        self.write_reg(Register::SOFT_RESET.addr(), val)
    }

    /// The OFSX, OFSY, and OFSZ registers are each eight bits and offer user set offset adjustments
    /// in twos complement format, with a scale factor of 3.9 mg/LSB (that is, 0x7F = 0.5g). The
    /// value stored in the offset registers is automatically added to the acceleration data,
    /// and the resulting value is stored in the output data registers.
    pub fn offsets(
        &mut self,
        offset_x: u8,
        offset_y: u8,
        offset_z: u8,
    ) -> Result<(), Adxl313Error<InterfaceError>> {
        self.write_reg(Register::OFSX.addr(), offset_x)?;
        self.write_reg(Register::OFSY.addr(), offset_y)?;
        self.write_reg(Register::OFSZ.addr(), offset_z)
    }

    /// The THRESH_ACT register is eight bits and holds the thresholdvalue for detecting activity.
    /// The data format is unsigned; therefore, the magnitude of the activity event is compared with
    /// the value in the THRESH_ACT register. The scale factor is 15.625 mg/LSB. A value of 0 may
    /// result in undesirable behavior if the activity interrupt is enabled.
    pub fn activity_threshold(
        &mut self,
        threshold: u8,
    ) -> Result<(), Adxl313Error<InterfaceError>> {
        self.write_reg(Register::THRESH_ACT.addr(), threshold)
    }

    /// The THRESH_INACT register is eight bits and holds the thresholdvalue for detecting
    /// inactivity. The data format is unsigned; therefore, the magnitude of the inactivity event
    /// is compared with the value in the THRESH_INACT register. The scale factor is 15.625 mg/LSB.
    /// A value of 0 may result in undesirable behavior if the inactivity interrupt is enabled.
    pub fn inactivity_threshold(
        &mut self,
        threshold: u8,
    ) -> Result<(), Adxl313Error<InterfaceError>> {
        self.write_reg(Register::THRESH_INACT.addr(), threshold)
    }

    /// The TIME_INACT register is eight bits and contains an unsignedtime value. Acceleration must
    /// be less than the value in the THRESH_INACT register for the amount of time represented by
    /// TIME_INACT for inactivity to be declared. The scale factor is 1 sec/LSB. Unlike the other
    /// interrupt functions, which use unfiltered data (see the Threshold section), the inactivity
    /// function uses filtered output data. At least one output sample must be generated for the
    /// inactivity interrupt to be triggered. This results in the function appearing unresponsive if
    /// the TIME_INACT register is set to a value less than the time constant of the output data
    /// rate. A value of 0 results in an interrupt when the output data is less than the value in
    /// the THRESH_INACT register.
    pub fn inactivity_time(&mut self, time: u8) -> Result<(), Adxl313Error<InterfaceError>> {
        self.write_reg(Register::TIME_INACT.addr(), time)
    }

    pub fn activity_inactivity_control(
        &mut self,
        activity_coupling: AcDcCoupling,
        inactivity_coupling: AcDcCoupling,
        enable_x_activity: bool,
        enable_x_inactivity: bool,
        enable_y_activity: bool,
        enable_y_inactivity: bool,
        enable_z_activity: bool,
        enable_z_inactivity: bool,
    ) -> Result<(), Adxl313Error<InterfaceError>> {
        let val = (activity_coupling.val() << 7) + (enable_x_activity as u8)
            << 6 + (enable_y_activity as u8)
            << 5 + (enable_z_activity as u8)
            << 4 + (inactivity_coupling.val() << 3) + (enable_x_inactivity as u8)
            << 2 + (enable_y_inactivity as u8)
            << 1 + (enable_z_inactivity as u8);
        self.write_reg(Register::ACT_INACT_CTL.addr(), val)
    }

    pub fn output_data_rate_and_low_power(
        &mut self,
        rate: OutputDataRate,
        low_power_enabled: bool,
    ) -> Result<(), Adxl313Error<InterfaceError>> {
        let val = ((low_power_enabled as u8) << 4) + rate.val();
        self.write_reg(Register::BW_RATE.addr(), val)?;
        self.output_data_rate = Some(rate);
        Ok(())
    }

    pub fn power_control(
        &mut self,
        i2c_disabled: bool,
        concurrent_activity_inactivity: bool,
        auto_sleep: bool,
        measuring: bool,
        sleep_enabled: bool,
        sleep_mode_frequency_reading: SleepModeFrequencyReadings,
    ) -> Result<(), Adxl313Error<InterfaceError>> {
        let val = ((i2c_disabled as u8) << 6)
            + ((concurrent_activity_inactivity as u8) << 5)
            + ((auto_sleep as u8) << 4)
            + ((measuring as u8) << 3)
            + ((sleep_enabled as u8) << 2)
            + sleep_mode_frequency_reading.val();
        self.write_reg(Register::POWER_CTL.addr(), val)
    }

    pub fn interrupt_enable(
        &mut self,
        conf: InterruptSource,
    ) -> Result<(), Adxl313Error<InterfaceError>> {
        self.write_reg(Register::INT_ENABLE.addr(), conf.value)
    }

    pub fn interrupt_pin_mapping(
        &mut self,
        conf: InterruptSource,
    ) -> Result<(), Adxl313Error<InterfaceError>> {
        self.write_reg(Register::INT_MAP.addr(), conf.value)
    }

    pub fn interrupt_source(&mut self) -> Result<InterruptSource, Adxl313Error<InterfaceError>> {
        self.read_reg(Register::INT_SOURCE.addr())
            .map(|value| InterruptSource { value })
    }

    pub fn data_format(
        &mut self,
        self_test: bool,
        spi_mode: SpiMode,
        irq_mode: IrqMode,
        full_resolution: bool,
        left_justified_data: bool,
        range: Range,
    ) -> Result<(), Adxl313Error<InterfaceError>> {
        let val: u8 = ((self_test as u8) << 7)
            + (spi_mode.val() << 6)
            + (irq_mode.val() << 5)
            + ((full_resolution as u8) << 3)
            + ((left_justified_data as u8) << 2)
            + range.val();
        self.write_reg(Register::DATA_FORMAT.addr(), val)?;
        self.range = Some(range);
        self.left_justified_data = left_justified_data;
        self.full_resolution = full_resolution;
        Ok(())
    }

    pub fn fifo_control(
        &mut self,
        fifo_mode: FifoMode,
        interrupt_to_pin2: bool,
        samples: u8,
    ) -> Result<(), Adxl313Error<InterfaceError>> {
        let samples = samples & 0b1_1111;
        let val = ((fifo_mode as u8) << 6) + ((interrupt_to_pin2 as u8) << 5) + samples;
        self.write_reg(Register::FIFO_CTL.addr(), val)
    }

    pub fn fifo_status(&mut self) -> Result<FifoStatus, Adxl313Error<InterfaceError>> {
        self.read_reg(Register::FIFO_STATUS.addr())
            .map(FifoStatus::new)
    }

    pub fn start_measuring(&mut self) -> Result<(), Adxl313Error<InterfaceError>> {
        let val = self.read_reg(Register::POWER_CTL.addr())?;
        if val & bitmask::power_ctl::SLEEP != 0 {
            // Device is sleeping. First put into standby mode
            self.write_reg(
                Register::POWER_CTL.addr(),
                val & !bitmask::power_ctl::MEASURE,
            )?;
            // Then put device out of sleep mode
            self.write_reg(Register::POWER_CTL.addr(), val & !bitmask::power_ctl::SLEEP)?;
        }
        self.write_reg(
            Register::POWER_CTL.addr(),
            val | bitmask::power_ctl::MEASURE,
        )?;

        let val = self.read_reg(Register::POWER_CTL.addr())?;

        Ok(())
    }

    pub fn is_10_bit(&self) -> bool {
        is_10_bit(
            self.output_data_rate.unwrap_or_default(),
            self.left_justified_data,
            self.full_resolution,
            self.range.unwrap_or_default(),
        )
    }

    /// Get the device ID
    pub fn get_device_id(&mut self) -> Result<u32, Adxl313Error<InterfaceError>> {
        // loop {
        //     let dev0 = self.read_reg(Register::DEVID_0.addr())?;
        //     // }
        //     // let dev0 = 0u8;
        //     let dev1 = self.read_reg(Register::DEVID_1.addr())?;
        //     let part_id = self.read_reg(Register::PARTID.addr())?;
        // }
        // loop {
        let dev0 = self.read_reg(Register::DEVID_0.addr())?;
        // }
        // let dev0 = 0u8;
        let dev1 = self.read_reg(Register::DEVID_1.addr())?;
        let part_id = self.read_reg(Register::PARTID.addr())?;

        Ok(((dev0 as u32) << 16) | ((dev1 as u32) << 8) | (part_id as u32))
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Adxl313Error<InterfaceError>> {
        self.interface
            .write(&mut [reg, value])
            .map_err(Adxl313Error::InterfaceError)
    }

    fn read_reg(&mut self, reg: u8) -> Result<u8, Adxl313Error<InterfaceError>> {
        let mut bytes = [0];
        self.interface
            .transfer(&[reg], &mut bytes)
            .map_err(Adxl313Error::InterfaceError)?;
        Ok(bytes[0])
    }
}

#[inline]
fn is_10_bit(
    odr: OutputDataRate,
    left_justified_data: bool,
    full_resolution: bool,
    range: Range,
) -> bool {
    let odr = odr.val();
    (odr == ODR_3200_HZ.val() || odr == ODR_1600_HZ.val())
        && left_justified_data
        && (full_resolution || range.val() == Range::_0d5G.val())
}

#[inline]
fn i32_from_2_u8_in_buf(
    buffer: &[u8],
    offset: usize,
    right_shift: u16,
    mask: u16,
    sign_mask: u16,
) -> i32 {
    let r = ((buffer[offset + 1] as u16) << 8) | (buffer[offset] as u16);
    let mut r = (r >> right_shift) & mask;
    if r & sign_mask > 0 {
        r = (r & !sign_mask) | (0b1000_0000_0000_0000);
    }
    (r as i16) as i32
}

const MASK_16BIT: u16 = 0b1111_1111_1111_1111;
const MASK_10BIT: u16 = 0b0000_0011_1111_1111;

#[inline]
fn x_y_z_raw_values(buffer: [u8; 6], range: Range, is_10_bit: bool) -> (i32, i32, i32) {
    let right_shift = if is_10_bit {
        6 - (range.val() as u16)
    } else {
        0
    };
    let mask = if is_10_bit { MASK_10BIT } else { MASK_16BIT };
    let sign_mask = 1 << (if is_10_bit { 9 } else { 15 });

    let x = i32_from_2_u8_in_buf(&buffer, 0, right_shift, mask, sign_mask);
    let y = i32_from_2_u8_in_buf(&buffer, 2, right_shift, mask, sign_mask);
    let z = i32_from_2_u8_in_buf(&buffer, 4, right_shift, mask, sign_mask);

    (x, y, z)
}

impl<IF, E> RawAccelerometer<I32x3> for Adxl313<IF>
where
    IF: Interface<Error = E>,
    E: Debug,
{
    type Error = E;

    /// Gets acceleration vector reading from the accelerometer
    /// Returns a 3D vector with x,y,z, fields in a Result
    fn accel_raw(&mut self) -> Result<I32x3, Error<E>> {
        // Must read all 6 registers at once, otherwise we'll be popping off the Fifo data
        // self.interface.write(&mut [Register::DATA_X0.addr()])?;
        let mut bytes = [0u8; 6];
        self.interface
            .transfer(&[Register::DATA_X0.addr()], &mut bytes)?;

        let is_10_bit = self.is_10_bit();
        let (x, y, z) = x_y_z_raw_values(bytes, self.range.unwrap_or_default(), is_10_bit);
        Ok(I32x3::new(x, y, z))
    }
}

impl<IF, E> Accelerometer for Adxl313<IF>
where
    IF: Interface<Error = E>,
    E: Debug,
{
    type Error = E;

    fn accel_norm(&mut self) -> Result<F32x3, Error<Self::Error>> {
        let raw_data: I32x3 = self.accel_raw()?;
        let range: f32 = self.range.unwrap_or_default().into(); // range in [g], so 0.5, 1, 2, 4 or 8

        let max = if self.is_10_bit() {
            ACCEL_MAX_10BIT
        } else {
            ACCEL_MAX_16BIT
        };

        let x = (raw_data.x as f32 / max as f32) * range;
        let y = (raw_data.y as f32 / max as f32) * range;
        let z = (raw_data.z as f32 / max as f32) * range;

        Ok(F32x3::new(x, y, z))
    }

    fn sample_rate(&mut self) -> Result<f32, Error<Self::Error>> {
        Ok(self.output_data_rate.unwrap_or_default().into())
    }
}

#[cfg(test)]
mod tests {
    use crate::OutputDataRate::{ODR_100_HZ, ODR_1600_HZ, ODR_50_HZ};
    use crate::{is_10_bit, x_y_z_raw_values, Range};

    #[test]
    fn test_x_y_z_raw() {
        // Given
        let buf: [u8; 6 + 1] = [
            0, // Command byte
            0b1111_1111,
            0b1111_1111, // X
            0b1111_1111,
            0b1111_1111, // Y
            0b1111_1111,
            0b1111_1111, // Z
        ];
        let buf2: [u8; 6 + 1] = [
            0,
            0b1010_0101,
            0b1010_0101, // X
            0b0101_1010,
            0b0101_1010, // Y
            0b1000_0001,
            0b1010_1100, // Z
        ];
        let is_10_bit_val = is_10_bit(ODR_100_HZ, false, false, Range::_0d5G);
        let (x, y, z) = x_y_z_raw_values(buf, Range::_0d5G, is_10_bit_val);
        assert_eq!(x, -1);
        assert_eq!(y, -1);
        assert_eq!(z, -1);

        let is_10_bit_val = is_10_bit(ODR_100_HZ, false, false, Range::_0d5G);
        let (x, y, z) = x_y_z_raw_values(buf2, Range::_0d5G, is_10_bit_val);
        assert_eq!(x, -23131);
        assert_eq!(y, 23130);
        assert_eq!(z, -21375);

        let is_10_bit_val = is_10_bit(ODR_50_HZ, true, true, Range::_2d0G);
        let (x, y, z) = x_y_z_raw_values(buf, Range::_2d0G, is_10_bit_val);
        assert_eq!(x, -1);
        assert_eq!(y, -1);
        assert_eq!(z, -1);

        let is_10_bit_val = is_10_bit(ODR_1600_HZ, true, true, Range::_0d5G);
        let (x, y, z) = x_y_z_raw_values(buf, Range::_0d5G, is_10_bit_val);
        assert_eq!(x, -32257);
        assert_eq!(y, -32257);
        assert_eq!(z, -32257);

        let is_10_bit_val = is_10_bit(ODR_1600_HZ, true, true, Range::_0d5G);
        let (x, y, z) = x_y_z_raw_values(buf2, Range::_0d5G, is_10_bit_val);
        assert_eq!(x, -32618); // 1010_0101 1010_0101 => 10_1001_0110 => 1000_0000_1001_0110
        assert_eq!(y, 361); // 0101_1010 0101_1010 => 01_0110_1001 => 0000_0001_0110_1001
        assert_eq!(z, -32590); // 1010_1100 1000_0001 => 10_1011_0010 => 1000_0000_1011_0010

        let is_10_bit_val = is_10_bit(ODR_1600_HZ, true, true, Range::_2d0G);
        let (x, y, z) = x_y_z_raw_values(buf2, Range::_2d0G, is_10_bit_val);
        assert_eq!(x, -32678); // 1010_0101 1010_0101 => 10_0101 1010 => 1000_0000_0101_1010
        assert_eq!(y, 421); // 0101_1010 0101_1010 => 01_1010_0101 => 0000_0001_1010_0101
        assert_eq!(z, -32568); // 1010_1100 1000_0001 => 10_1100_1000 => 1000_0000_1100_1000
    }

    #[test]
    fn cast_looks_at_the_bits_not_the_value() {
        let b: u8 = 0b1000_0000;
        let c = b as i8;
        let d: i8 = -128;
        assert_eq!(c, d);
        assert_eq!(b, 128)
    }
}
