"""BMP280 I2C Driver - Barometric pressure and temperature sensor."""

import random
import struct


class FakeBMP280Driver:
    """Fake driver that generates random pressure/temperature data without I2C hardware."""

    def __init__(self, **kwargs):
        pass

    def read_all(self):
        """Return (pressure_pa, temperature_c)."""
        pressure = 101325.0 + random.gauss(0.0, 50.0)      # ~1013.25 hPa
        temperature = 25.0 + random.gauss(0.0, 0.5)         # ~25 °C
        return pressure, temperature

    def chip_id(self) -> int:
        return 0x58

    def close(self):
        pass


class BMP280Driver:
    """Low-level I2C driver for Bosch BMP280.

    Datasheet: BST-BMP280-DS001
    Compensation formulas follow the double-precision floating-point variant
    described in Section 8.1 of the datasheet.
    """

    # ── Register Map ──────────────────────────────────────────────
    REG_CALIB00    = 0x88   # calibration data (26 bytes: 0x88-0xA1)
    REG_CHIP_ID    = 0xD0   # should read 0x58
    REG_RESET      = 0xE0   # write 0xB6 for soft reset
    REG_STATUS     = 0xF3
    REG_CTRL_MEAS  = 0xF4
    REG_CONFIG     = 0xF5
    REG_PRESS_MSB  = 0xF7   # 3 bytes: press[19:12], [11:4], [3:0]<<4
    REG_TEMP_MSB   = 0xFA   # 3 bytes: temp[19:12], [11:4], [3:0]<<4

    def __init__(self, bus: int = 1, address: int = 0x76,
                 pressure_oversampling: int = 5,
                 temperature_oversampling: int = 2,
                 iir_filter: int = 4):
        from smbus2 import SMBus

        self.address = address
        self.bus = SMBus(bus)
        self._t_fine = 0.0

        self._read_calibration()
        self._configure(pressure_oversampling, temperature_oversampling,
                        iir_filter)

    # ── Calibration ──────────────────────────────────────────────
    def _read_calibration(self):
        """Read factory calibration coefficients from registers 0x88-0xA1."""
        raw = bytes(self.bus.read_i2c_block_data(
            self.address, self.REG_CALIB00, 26))

        # Temperature coefficients
        self.dig_T1 = struct.unpack_from('<H', raw, 0)[0]
        self.dig_T2 = struct.unpack_from('<h', raw, 2)[0]
        self.dig_T3 = struct.unpack_from('<h', raw, 4)[0]

        # Pressure coefficients
        self.dig_P1 = struct.unpack_from('<H', raw, 6)[0]
        self.dig_P2 = struct.unpack_from('<h', raw, 8)[0]
        self.dig_P3 = struct.unpack_from('<h', raw, 10)[0]
        self.dig_P4 = struct.unpack_from('<h', raw, 12)[0]
        self.dig_P5 = struct.unpack_from('<h', raw, 14)[0]
        self.dig_P6 = struct.unpack_from('<h', raw, 16)[0]
        self.dig_P7 = struct.unpack_from('<h', raw, 18)[0]
        self.dig_P8 = struct.unpack_from('<h', raw, 20)[0]
        self.dig_P9 = struct.unpack_from('<h', raw, 22)[0]

    # ── Configuration ────────────────────────────────────────────
    def _configure(self, press_os: int, temp_os: int, iir_filter: int):
        """Set oversampling and IIR filter, then switch to normal mode."""
        # CONFIG register: t_sb[7:5]=0b000 (0.5 ms), filter[4:2], spi3w_en=0
        config_val = (iir_filter & 0x07) << 2
        self.bus.write_byte_data(self.address, self.REG_CONFIG, config_val)

        # CTRL_MEAS: osrs_t[7:5], osrs_p[4:2], mode[1:0]=0b11 (normal)
        ctrl_val = ((temp_os & 0x07) << 5) | ((press_os & 0x07) << 2) | 0x03
        self.bus.write_byte_data(self.address, self.REG_CTRL_MEAS, ctrl_val)

    # ── Raw read ─────────────────────────────────────────────────
    def _read_raw(self):
        """Burst-read 6 bytes (press + temp) starting at 0xF7."""
        buf = bytes(self.bus.read_i2c_block_data(
            self.address, self.REG_PRESS_MSB, 6))

        adc_P = ((buf[0] << 16) | (buf[1] << 8) | buf[2]) >> 4
        adc_T = ((buf[3] << 16) | (buf[4] << 8) | buf[5]) >> 4

        return adc_P, adc_T

    # ── Compensation (datasheet Section 8.1, double precision) ───
    def _compensate_temperature(self, adc_T: int) -> float:
        """Return compensated temperature in degrees Celsius."""
        var1 = (adc_T / 16384.0 - self.dig_T1 / 1024.0) * self.dig_T2
        var2 = ((adc_T / 131072.0 - self.dig_T1 / 8192.0) ** 2) * self.dig_T3
        self._t_fine = var1 + var2
        return self._t_fine / 5120.0

    def _compensate_pressure(self, adc_P: int) -> float:
        """Return compensated pressure in Pascals.

        Must be called AFTER _compensate_temperature (needs _t_fine).
        """
        var1 = self._t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * self.dig_P6 / 32768.0
        var2 = var2 + var1 * self.dig_P5 * 2.0
        var2 = var2 / 4.0 + self.dig_P4 * 65536.0
        var1 = (self.dig_P3 * var1 * var1 / 524288.0 + self.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1

        if var1 == 0.0:
            return 0.0

        pressure = 1048576.0 - adc_P
        pressure = (pressure - var2 / 4096.0) * 6250.0 / var1
        var1 = self.dig_P9 * pressure * pressure / 2147483648.0
        var2 = pressure * self.dig_P8 / 32768.0
        pressure = pressure + (var1 + var2 + self.dig_P7) / 16.0

        return pressure

    # ── Public API ────────────────────────────────────────────────
    def read_all(self):
        """Read pressure and temperature in one burst.

        Returns:
            pressure    – barometric pressure in Pa
            temperature – temperature in degrees Celsius
        """
        adc_P, adc_T = self._read_raw()
        temperature = self._compensate_temperature(adc_T)
        pressure = self._compensate_pressure(adc_P)
        return pressure, temperature

    def chip_id(self) -> int:
        """Read CHIP_ID register (should return 0x58 for genuine BMP280)."""
        return self.bus.read_byte_data(self.address, self.REG_CHIP_ID)

    def close(self):
        self.bus.close()
