from Adafruit_I2C import Adafruit_I2C
from constants import *

class I2C(object):
    
    def __init__(self, busId = 2):
        self._i2c = Adafruit_I2C(LSM6DS33_ADDR, busId)
        
    def __del__(self):
        try:
            del(self._i2c)
        except:
            pass
        
    def _combineLoHi(self, loByte, hiByte):
        return (loByte | hiByte << 8)
        
    def _combineSignedLoHi(self, loByte, hiByte):
        combined = self._combineLoHi(loByte, hiByte)
        return combined if combined < 32768 else (combined - 65536)
        
    def _getSensorRawLoHi3(self, outRegs):
        xl = self._readRegister(outRegs[0])
        xh = self._readRegister(outRegs[1])
        yl = self._readRegister(outRegs[2])
        yh = self._readRegister(outRegs[3])
        zl = self._readRegister(outRegs[4])
        zh = self._readRegister(outRegs[5])
        
        xVal = self._combineSignedLoHi(xl, xh)
        yVal = self._combineSignedLoHi(yl, yh)
        zVal = self._combineSignedLoHi(zl, zh)
        
        return [xVal, yVal, zVal]
        
    def _readRegister(self, register):
        return self._i2c.readS8(register)
    
    def _writeRegister(self, register, value):
        valueOld = self._readRegister(register)
        self._i2c.write8(register, value)
        return valueOld