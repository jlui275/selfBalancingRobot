import math

from constants import *
from lsm6ds33 import LSM6DS33

#class for the IMU
class AltIMU(LSM6DS33):
    #private methods
    def __init__(self):
        super(AltIMU, self).__init__()
        
        #initialize tracked gyroscope angles
        self.gyrAngleX = 0.0
        self.gyrAngleY = 0.0
        self.gyrAngleZ = 0.0
        
        ## Initialize Kalman filter variables
        # Bias values for X, Y, and Z
        self.kalmanBiasX = 0.0
        self.kalmanBiasY = 0.0
        self.kalmanBiasZ = 0.0
        # State vectors for X, Y, and Z (XP_00, XP_01, XP_10, XP_11)
        self.kalmanXP_00 = self.kalmanXP_01 = 0.0
        self.kalmanXP_10 = self.kalmanXP_11 = 0.0
        self.kalmanYP_00 = self.kalmanYP_01 = 0.0
        self.kalmanYP_10 = self.kalmanYP_11 = 0.0
        self.kalmanZP_00 = self.kalmanZP_01 = 0.0
        self.kalmanZP_10 = self.kalmanZP_11 = 0.0
        # Kalman filtered angle values
        self.kalmanAngleX = 0.0
        self.kalmanAngleY = 0.0
        self.kalmanAngleZ = 0.0
        
        #initialize complementary filter variables
        self.complementaryAngleX = 0.0
        self.complementaryAngleY = 0.0
        self.complementaryAngleZ = 0.0
    
    def __del__(self):
        super(AltIMU, self).__del__()
    
    def enable(self, accelerometer = True, gyroscope = True):
        if accelerometer or gyroscope:
            self.enableLSM(accelerometer = accelerometer, gyroscope = gyroscope)
            if gyroscope:
                self.calibrateGyroAngles()
    
    def calibrateGyroAngles(self, xCal = 0.0, yCal = 0.0, zCal = 0.0):
        self.gyrAngles = [xCal, yCal, zCal]
        
    #get gyro rotation rates:
    def getGyroRotationRates(self):
        #get raw data from gyroscope
        [gyrRawX, gyrRawY, gyrRawZ] = self.getGyroscopeRaw()
        
        gyrRateX = gyrRawX * GYRO_GAIN
        gyrRateY = gyrRawY * GYRO_GAIN
        gyrRateZ = gyrRawZ * GYRO_GAIN
        
        return [gyrRateX, gyrRateY, gyrRateZ]
        
    #track gyro angles
    def trackGyroAngles(self, deltaT = 0.02):
        #get current gyro rotation rate
        [gyrRateX, gyrRateY, gyrRateZ] = self.getGyroRotationRates()
        
        #sum up and multiply by angle tracking
        self.gyrAngleX += gyrRateX * deltaT
        self.gyrAngleY += gyrRateY * deltaT
        self.gyrAngleZ += gyrRateZ * deltaT
        
        #return [round(self.gyrAngleX, 3), round(self.gyrAngleY, 3), round(self.gyrAngleZ, 3)]
        return [self.gyrAngleX, self.gyrAngleY, self.gyrAngleZ]
    
    #get accelerometer angles
    def getAccelerometerAngles(self):
        [accXRaw, accYRaw, accZRaw] = self.getAccelerometerRaw()
        
        #calculate angles
        accXAngle = math.degrees(math.atan2(accYRaw, accZRaw) + math.pi)
        accYAngle = math.degrees(math.atan2(accXRaw, accZRaw) + math.pi)
        accZAngle = math.degrees(math.atan2(accXRaw, accYRaw) + math.pi)
        
        #return vector of angle values
        #return [round(accXAngle, 3), round(accYAngle, 3), round(accZAngle, 3)]
        return [accXAngle, accYAngle, accZAngle]
    
    def getComplementaryAngles(self, deltaT = 0.05):
        """ Calculate combined angles of accelerometer and gyroscope
            using a complementary filter.
            Note: This filter is very cheap CPU-wise, but the result
            follows the drift of the gyroscope.
        """
        # If accelerometer or gyroscope is not enabled or none of the
        # dimensions is requested make a quick turnaround
        #if not (self.accelerometer and self.gyroscope and (x or y or z)):
        #    return (None, None, None)
        
        # Get gyroscope rotation rates and accelerometer angles
        gyrRates = self.getGyroRotationRates()
        accelAngles = self.getAccelerometerAngles()

        # Determine wether to initialize the complementary filter angles
        # from the accelerometer readings in the first iteration
        if self.initComplementaryFromAccel:
            self.complementaryAngles = list(accelAngles)
            self.initComplementaryFromAccel = False

        # Calculate complementary filtered angles
        self.complementaryAngles = [None if (gyrRates[i] is None or accelAngles[i] is None)
            else C_FILTER_CONST * (self.complementaryAngles[i] + gyrRates[i] * deltaT)
            + (1 - C_FILTER_CONST) * accelAngles[i]
            for i in range(3)]

        # Return vector
        return tuple(self.complementaryAngles)
    
    def getKalmanAngles(self, deltaT = 0.05):
        def _calculateKalmanAngle(kalmanP_00,
                                  kalmanP_01,
                                  kalmanP_10,
                                  kalmanP_11,
                                  gyrRate,
                                  accAngle,
                                  kalmanBias,
                                  kalmanAngle,
                                  deltaT):
            """ Calculate Kalman filtered angle and return updated filter
                matrix for one dimension.
            """

            # Gyroscope part
            kalmanAngle += (gyrRate - kalmanBias) * deltaT

            kalmanP_00 += -deltaT * (kalmanP_01 + kalmanP_10) + K_Q_ANGLE * deltaT
            kalmanP_01 += -deltaT * kalmanP_11
            kalmanP_10 += -deltaT * kalmanP_11
            kalmanP_11 += K_Q_GYRO * deltaT

            # Accelerometer part
            kalY = accAngle - kalmanAngle
            kalS = kalmanP_00 + K_R_ANGLE
            kal0 = kalmanP_00 / kalS
            kal1 = kalmanP_10 / kalS

            # Set Kalman filtered angle
            kalmanAngle +=  kal0 * kalY

            # Re-calculate Kalman parameters
            kalmanBias +=  kal1 * kalY
            kalmanP_00 -= kal0 * kalmanP_00
            kalmanP_01 -= kal0 * kalmanP_01
            kalmanP_10 -= kal1 * kalmanP_10
            kalmanP_11 -= kal1 * kalmanP_11

            return (kalmanP_00, kalmanP_01, kalmanP_10, kalmanP_11,
                    kalmanBias, kalmanAngle)

        # If accelerometer or gyroscope is not enabled or none of the
        # dimensions is requested make a quick turnaround
        #if not (self.accelerometer and self.gyroscope and (x or y or z)):
        #    return (None, None, None)

        # Get gyroscope rotation rates and accelerometer angles
        [gyrRateX, gyrRateY, gyrRateZ] = self.getGyroRotationRates()
        [accAngleX, accAngleY, accAngleZ] = self.getAccelerometerAngles()

        # Determine wether to initialize the Kalman angles from the
        # accelerometer readings in the first iteration
        #if self.initKalmanFromAccel:
        #    self.kalmanAngles = list(accelAngles)
        #    self.initKalmanFromAccel = False

        # X axis
        (self.kalmanXP_00,
         self.kalmanXP_01,
         self.kalmanXP_10,
         self.kalmanXP_11,
         self.kalmanBiasX,
         self.kalmanAngleX) = _calculateKalmanAngle(accAngleX,
                                               gyrRateX,
                                               self.kalmanXP_00,
                                               self.kalmanXP_01,
                                               self.kalmanXP_10,
                                               self.kalmanXP_11,
                                               self.kalmanBiasX,
                                               self.kalmanAngleX,
                                               deltaT)

        # Y axis
        (self.kalmanYP_00,
         self.kalmanYP_01,
         self.kalmanYP_10,
         self.kalmanYP_11,
         self.kalmanBiasY,
         self.kalmanAngleY) = _calculateKalmanAngle(accAngleY,
                                               gyrRateY,
                                               self.kalmanYP_00,
                                               self.kalmanYP_01,
                                               self.kalmanYP_10,
                                               self.kalmanYP_11,
                                               self.kalmanBiasY,
                                               self.kalmanAngleY,
                                               deltaT)
        # Z axis
        (self.kalmanZP_00,
         self.kalmanZP_01,
         self.kalmanZP_10,
         self.kalmanZP_11,
         self.kalmanBiasZ,
         self.kalmanAngleZ) = _calculateKalmanAngle(accAngleZ,
                                               gyrRateZ,
                                               self.kalmanZP_00,
                                               self.kalmanZP_01,
                                               self.kalmanZP_10,
                                               self.kalmanZP_11,
                                               self.kalmanBiasZ,
                                               self.kalmanAngleZ,
                                               deltaT)

        # Return vector
        return [round(self.kalmanAngleX, 5), round(self.kalmanAngleY, 5), round(self.kalmanAngleZ, 5)]