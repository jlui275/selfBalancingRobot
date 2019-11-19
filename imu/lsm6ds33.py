from i2c import I2C

from constants import *

class LSM6DS33(I2C):
    ##
    ## Class variables and constants
    ##

    # Register addresses
    #  ([+] = used in the code, [-] = not used or useful, [ ] = TBD)
    LSM_FUNC_CFG_ACCESS   = 0x01  # [-] Configuration of embedded
                                  #     functions, e.g. pedometer

    LSM_FIFO_CTRL1        = 0x06  # [-] FIFO threshold setting
    LSM_FIFO_CTRL2        = 0x07  # [-] FIFO control register
    LSM_FIFO_CTRL3        = 0x08  # [-] Gyro/Acceleromter-specific FIFO settings
    LSM_FIFO_CTRL4        = 0x09  # [-] FIFO data storage control
    LSM_FIFO_CTRL5        = 0x0A  # [-] FIFO ODR/Mode selection

    LSM_ORIENT_CFG_G      = 0x0B  # [ ] Gyroscope sign/orientation

    LSM_INT1_CTRL         = 0x0D  # [-] INT1 pad control - unavailable for AltIMU
    LSM_INT2_CTRL         = 0x0E  # [-] INT2 pad control - unavailable for AltIMU
    LSM_WHO_AM_I          = 0x0F  # [-] Returns 0x69 (read only)
    LSM_CTRL1_XL          = 0x10  # [+] Acceleration sensor control
    LSM_CTRL2_G           = 0x11  # [+] Angular rate sensor (gyroscope) control
    LSM_CTRL3_C           = 0x12  # [+] Device/communication settings
    LSM_CTRL4_C           = 0x13  # [ ] Bandwith/sensor/communication settings
    LSM_CTRL5_C           = 0x14  # [ ] Rounding/self-test control
    LSM_CTRL6_C           = 0x15  # [ ] Gyroscope settings
    LSM_CTRL7_G           = 0x16  # [ ] Gyroscope settings
    LSM_CTRL8_XL          = 0x17  # [ ] Acceleration sensor settings
    LSM_CTRL9_XL          = 0x18  # [ ] Acceleration sensor axis control
    LSM_CTRL10_C          = 0x19  # [ ] Gyroscope axis control / misc. settings

    LSM_WAKE_UP_SRC       = 0x1B  # [-] Wake up interrupt source register
    LSM_TAP_SRC           = 0x1C  # [-] Tap source register
    LSM_D6D_SRC           = 0x1D  # [-] Orientation sensing for Android devices

    LSM_STATUS_REG        = 0x1E  # [ ] Status register. Shows if new data
                                  #     is available from one or more of the
                                  #     sensors

    LSM_OUT_TEMP_L        = 0x20  # [+] Temperature output, low byte
    LSM_OUT_TEMP_H        = 0x21  # [+] Temperature output, high byte
    LSM_OUTX_L_G          = 0x22  # [+] Gyroscope X output, low byte
    LSM_OUTX_H_G          = 0x23  # [+] Gyroscope X output, high byte
    LSM_OUTY_L_G          = 0x24  # [+] Gyroscope Y output, low byte
    LSM_OUTY_H_G          = 0x25  # [+] Gyroscope Y output, high byte
    LSM_OUTZ_L_G          = 0x26  # [+] Gyroscope Z output, low byte
    LSM_OUTZ_H_G          = 0x27  # [+] Gyroscope Z output, high byte
    LSM_OUTX_L_XL         = 0x28  # [+] Accelerometer X output, low byte
    LSM_OUTX_H_XL         = 0x29  # [+] Accelerometer X output, high byte
    LSM_OUTY_L_XL         = 0x2A  # [+] Accelerometer Y output, low byte
    LSM_OUTY_H_XL         = 0x2B  # [+] Accelerometer Y output, high byte
    LSM_OUTZ_L_XL         = 0x2C  # [+] Accelerometer Z output, low byte
    LSM_OUTZ_H_XL         = 0x2D  # [+] Accelerometer Z output, high byte

    LSM_FIFO_STATUS1      = 0x3A  # [-] Number of unread words in FIFO
    LSM_FIFO_STATUS2      = 0x3B  # [-] FIFO status control register
    LSM_FIFO_STATUS3      = 0x3C  # [-] FIFO status control register
    LSM_FIFO_STATUS4      = 0x3D  # [-] FIFO status control register
    LSM_FIFO_DATA_OUT_L   = 0x3E  # [-] FIFO data output, low byte
    LSM_FIFO_DATA_OUT_H   = 0x3F  # [-] FIFO data output, high byte

    LSM_TIMESTAMP0_REG    = 0x40  # [-] Time stamp first byte data output
    LSM_TIMESTAMP1_REG    = 0x41  # [-] Time stamp second byte data output
    LSM_TIMESTAMP2_REG    = 0x42  # [-] Time stamp third byte data output

    LSM_STEP_TIMESTAMP_L  = 0x49  # [-] Time stamp of last step (for pedometer)
    LSM_STEP_TIMESTAMP_H  = 0x4A  # [-] Time stamp of last step, high byte
    LSM_STEP_COUNTER_L    = 0x4B  # [-] Step counter output, low byte
    LSM_STEP_COUNTER_H    = 0x4C  # [-] Step counter output, high byte

    LSM_FUNC_SRC          = 0x53  # [-] Interrupt source register for
                              #     embedded functions

    LSM_TAP_CFG           = 0x58  # [-] Configuration of embedded functions
    LSM_TAP_THS_6D        = 0x59  # [-] Orientation and tap threshold
    LSM_INT_DUR2          = 0x5A  # [-] Tap recognition settings
    LSM_WAKE_UP_THS       = 0x5B  # [-] Wake up threshold settings
    LSM_WAKE_UP_DUR       = 0x5C  # [-] Wake up function settings
    LSM_FREE_FALL         = 0x5D  # [-] Free fall duration settings
    LSM_MD1_CFG           = 0x5E  # [-] Function routing for INT1
    LSM_MD2_CFG           = 0x5F  # [-] Function routing for INT2

    # Output registers used by the accelerometer
    lsmAccRegisters = [
        LSM_OUTX_L_XL,       # low byte of X value
        LSM_OUTX_H_XL,       # high byte of X value
        LSM_OUTY_L_XL,       # low byte of Y value
        LSM_OUTY_H_XL,       # high byte of Y value
        LSM_OUTZ_L_XL,       # low byte of Z value
        LSM_OUTZ_H_XL,       # high byte of Z value
    ]

    # Output registers used by the gyroscope
    lsmGyroRegisters = [
        LSM_OUTX_L_G,       # low byte of X value
        LSM_OUTX_H_G,       # high byte of X value
        LSM_OUTY_L_G,       # low byte of Y value
        LSM_OUTY_H_G,       # high byte of Y value
        LSM_OUTZ_L_G,       # low byte of Z value
        LSM_OUTZ_H_G,       # high byte of Z value
    ]

    # Output registers used by the temperature sensor
    lsmTempRegisters = [
        LSM_OUT_TEMP_L,     # low byte of temperature value
        LSM_OUT_TEMP_H,     # high byte of temperature value
    ]
    
    ##
    ## Class methods
    ##

    ## Private methods
    def __init__(self, busId = 2):
        super(LSM6DS33, self).__init__(busId)
        self.accEnabled = False
        self.gyroEnabled = False
        
    def __del__(self):
        """ Clean up routines. """
        try:
            # Power down accelerometer
            self._writeRegister(LSM6DS33_ADDR, self.LSM_CTRL1_XL, 0x00)
            # Power down gyroscope
            self._writeRegister(LSM6DS33_ADDR, self.LSM_CTRL2_G, 0x00)
            super(LSM6DS33, self).__del__()
        except:
            pass
    
    #enable the accelerometer and gyroscope
    def enableLSM(self, accelerometer = True, gyroscope = True):
        # disable accelerometer and gyroscope first
        self._writeRegister(self.LSM_CTRL1_XL, 0x00)
        self._writeRegister(self.LSM_CTRL2_G, 0x00)
        self._writeRegister(self.LSM_CTRL3_C, 0x00)
        
        #Initialize Flags
        self.accEnabled = False
        self.gyroEnabled = False
        
        #disable FIFO
        self._writeRegister(self.LSM_FIFO_CTRL5, 0x00)
        
        if accelerometer:
            #enable accelerometer
            # 1.66 kHz / +/- 4g
            self._writeRegister(self.LSM_CTRL1_XL, 0x58)
            self.accEnabled = True
            print("Accelerometer Enabled")
        
        if gyroscope:
            #enable gyroscope
            # 208 Hz high performance / 1000 dps
            self._writeRegister(self.LSM_CTRL2_G, 0x58)
            self.gyroEnabled = True
            print("Gyroscope Enabled")
   
    #get raw accelerometer data
    def getAccelerometerRaw(self):
        if not self.accEnabled:
            raise(Exception("Accelerometer has to be enabled first"))
        """#read sensor data
        self.lsmAccRegisters[0] = i2c.readS8(LSM_OUTX_L_XL)      
        self.lsmAccRegisters[1] = i2c.readS8(LSM_OUTX_H_XL)        
        self.lsmAccRegisters[2] = i2c.readS8(LSM_OUTY_L_XL)        
        self.lsmAccRegisters[3] = i2c.readS8(LSM_OUTY_H_XL)        
        self.lsmAccRegisters[4] = i2c.readS8(LSM_OUTZ_L_XL)        
        self.lsmAccRegisters[5] = i2c.readS8(LSM_OUTZ_H_XL) 
        
        LSM_OUTX_XL = combineLoHiByte(lsmAccRegisters[0], lsmAccRegisters[1])
        LSM_OUTY_XL = combineLoHiByte(lsmAccRegisters[2], lsmAccRegisters[3])
        LSM_OUTZ_XL = combineLoHiByte(lsmAccRegisters[4], lsmAccRegisters[5])
        
        lsmAccRegistersCombined = [
            LSM_OUTX_XL,
            LSM_OUTY_XL,
            LSM_OUTZ_XL,
        ]
        return lsmAccRegistersCombined"""
        
        outReg = [self.LSM_OUTX_L_XL, self.LSM_OUTX_H_XL, self.LSM_OUTY_L_XL, self.LSM_OUTY_H_XL, self.LSM_OUTZ_L_XL, self.LSM_OUTZ_H_XL]
        return self._getSensorRawLoHi3(outReg)
        
    #get gyroscope data
    def getGyroscopeRaw(self):
        if not self.gyroEnabled:
            raise(Exception("Gyroscope has to be enabled first")) 
    
        """
        lsmGyroRegisters[0] = i2c.readS8(LSM_OUTX_L_G)      
        lsmGyroRegisters[1] = i2c.readS8(LSM_OUTX_H_G)       
        lsmGyroRegisters[2] = i2c.readS8(LSM_OUTY_L_G)       
        lsmGyroRegisters[3] = i2c.readS8(LSM_OUTY_H_G)       
        lsmGyroRegisters[4] = i2c.readS8(LSM_OUTZ_L_G)       
        lsmGyroRegisters[5] = i2c.readS8(LSM_OUTZ_H_G) 
        
        LSM_OUTX_G = combineLoHiByte(lsmGyroRegisters[0], lsmGyroRegisters[1])
        LSM_OUTY_G = combineLoHiByte(lsmGyroRegisters[2], lsmGyroRegisters[3])
        LSM_OUTZ_G = combineLoHiByte(lsmGyroRegisters[4], lsmGyroRegisters[5])
        
        lsmGyroRegistersCombined = [
            LSM_OUTX_G,
            LSM_OUTY_G,
            LSM_OUTZ_G,
        ]
        
        return lsmGyroRegistersCombined"""
        outReg = [self.LSM_OUTX_L_G, self.LSM_OUTX_H_G, self.LSM_OUTY_L_G, self.LSM_OUTY_H_G, self.LSM_OUTZ_L_G, self.LSM_OUTZ_H_G]
        return self._getSensorRawLoHi3(outReg)