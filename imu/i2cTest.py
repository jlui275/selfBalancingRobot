from Adafruit_I2C import Adafruit_I2C

LSM6DS33_ADDR = 0x6b

GYRO_GAIN = 0.035

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


i2c = Adafruit_I2C(LSM6DS33_ADDR, 2)

#enable the accelerometer and gyroscope
def enableLSM():
    # disable accelerometer and gyroscope first
    i2c.write8(LSM_CTRL1_XL, 0x00)
    i2c.write8(LSM_CTRL2_G, 0x00)
    i2c.write8(LSM_CTRL3_C, 0x00)
    
    #disable FIFO
    i2c.write8(LSM_FIFO_CTRL5, 0x00)
    
    #enable accelerometer
    # 1.66 kHz / +/- 4g
    i2c.write8(LSM_CTRL1_XL, 0x58)
    print("Accelerometer Enabled")
    
    #enable gyroscope
    # 208 Hz high performance / 1000 dps
    i2c.write8(LSM_CTRL2_G, 0x58)
    print("Gyroscope Enabled")

#combine high byte and low byte
def combineLoHiByte(loByte, hiByte):
    combinedBytes = (loByte | hiByte << 8)
    if(combinedBytes < 32768):
        return combinedBytes
    else:
        return combinedBytes - 65536
    
#get accelerometer data
def getAccelerometerRaw():
    lsmAccRegisters[0] = i2c.readS8(LSM_OUTX_L_XL)      
    lsmAccRegisters[1] = i2c.readS8(LSM_OUTX_H_XL)        
    lsmAccRegisters[2] = i2c.readS8(LSM_OUTY_L_XL)        
    lsmAccRegisters[3] = i2c.readS8(LSM_OUTY_H_XL)        
    lsmAccRegisters[4] = i2c.readS8(LSM_OUTZ_L_XL)        
    lsmAccRegisters[5] = i2c.readS8(LSM_OUTZ_H_XL) 
    
    LSM_OUTX_XL = combineLoHiByte(lsmAccRegisters[0], lsmAccRegisters[1])
    LSM_OUTY_XL = combineLoHiByte(lsmAccRegisters[2], lsmAccRegisters[3])
    LSM_OUTZ_XL = combineLoHiByte(lsmAccRegisters[4], lsmAccRegisters[5])
    
    lsmAccRegistersCombined = [
        LSM_OUTX_XL,
        LSM_OUTY_XL,
        LSM_OUTZ_XL,
    ]
    return lsmAccRegistersCombined
    
#get gyroscope data
def getGyroscopeRaw():
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
    
    return lsmGyroRegistersCombined

#get accelerometer angles
def getAccelerometerAngles(rawAccelerometerData):
    accXRaw = rawAccelerometerData[0]
    accYRaw = rawAccelerometerData[1]
    accZRaw = rawAccelerometerData[2]
    
    #calculate angles
    accXAngle = math.degrees(math.atan2(accYRaw, accZRaw) + math.pi)
    accYAngle = math.degrees(math.atan2(accXRaw, accZRaw) + math.pi)
    accZAngle = math.degrees(math.atan2(accXRaw, accYRaw) + math.pi)
    
    #return vector of angle values
    return [accXAngle, accYAngle, accZAngle]
    

#get gyro rotation rates:
def getGyroRotationRates(rawGyroscopeData):
    #get raw data from gyroscope
    gyrXRaw = rawGyroscopeData[0]
    gyrYRaw = rawGyroscopeData[1]
    gyrZRaw = rawGyroscopeData[2]
    
    gyrRateX = gyrRawX * GYRO_GAIN
    gyrRateY = gyrRawY * GYRO_GAIN
    gyrRateZ = gyrRawZ * GYRO_GAIN
    
    return [gyrRateX, gyrRateY, gyrRateZ]
    
#track gyro angles
def trackGyroAngles(deltaT = 0.02):
    #get current gyro rotation rate
    [gyrRateX, gyrRateY, gyrRateZ] = getGyroRotationRates(rawGryoscopeData)
    
    #sum up and multiply by angle tracking
    gyrAngleX += gyrRateX * deltaT
    gyrAngleY += gyrRateY * deltaT
    gyrAngleZ += gyrRateZ * deltaT
    
    return [gyrAngleX, gyrAngleY, gyrAngleZ]
enableLSM()
accelData = getAccelerometerRaw()
gyroData = getGyroscopeRaw()

print("Accelerometer Data: {}".format(accelData))
print("Gyroscope Data: {}".format(gyroData))