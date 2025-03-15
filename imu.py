from pyb import I2C
from ustruct import unpack_from, calcsize, pack

class BNO055:
    DEV_ADDR = 0x28
    NDOF_MODE = 0x0C

    class reg:
        EUL_HEADING_MSB = (0x1B, b"<B")
        EUL_HEADING_LSB = (0x1A, b"<B")
        EUL_HEADING     = (0x1A, b"<h")
        EUL_DATA_ALL    = (0x1A, b"<hhh")
        TEMP            = (0x34, b"<b")
        OPR_MODE        = (0x3D, b"<B")
        CALIB_STAT      = (0x35, b"<B")
        CALIB_COEFFS    = (0x55, b"<22B")
        GYRO_DATA       = (0x14, b"<hhh")

    def __init__(self, i2c):
        self._buf = bytearray((0 for n in range(22)))
        self._i2c = i2c

    def _read_reg(self, reg):
        length = calcsize(reg[1])
        buf = memoryview(self._buf)[:length]
        self._i2c.mem_read(buf, BNO055.DEV_ADDR, reg[0])
        return unpack_from(reg[1], buf)

    def _write_reg(self, reg, value):
        buf = bytearray(pack(reg[1], value))
        self._i2c.mem_write(buf, BNO055.DEV_ADDR, reg[0])

    def set_mode(self, mode):
        self._write_reg(BNO055.reg.OPR_MODE, mode)

    def read_cal_status(self):
        cal_status = self._read_reg(BNO055.reg.CALIB_STAT)[0]
        return {
            'sys': (cal_status >> 6) & 3,
            'gyro': (cal_status >> 4) & 3,
            'accel': (cal_status >> 2) & 3,
            'mag': cal_status & 3
        }

    def get_calibration_coeffs(self):
        return self._read_reg(BNO055.reg.CALIB_COEFFS)

    def set_calibration_coeffs(self, coeffs):
        for i, coeff in enumerate(coeffs):
            self._write_reg((BNO055.reg.CALIB_COEFFS[0] + i, b"<B"), coeff)

    def euler(self):
        head, roll, pitch = self._read_reg(BNO055.reg.EUL_DATA_ALL)
        return (head / 16, roll / 16, pitch / 16)

    def heading(self):
        return self._read_reg(BNO055.reg.EUL_HEADING)[0] / 16

    def roll(self):
        return self._read_reg(BNO055.reg.EUL_DATA_ALL)[1] / 16

    def pitch(self):
        return self._read_reg(BNO055.reg.EUL_DATA_ALL)[2] / 16

    def read_angular_velocity(self):
        x, y, z = self._read_reg(BNO055.reg.GYRO_DATA)
        return (x / 16, y / 16, z / 16)

    def yaw_rate(self):
        return self._read_reg(BNO055.reg.GYRO_DATA)[2] / 16