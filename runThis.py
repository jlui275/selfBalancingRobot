#!/usr/bin/python


from RoboClawAdvance import RoboClawAdvance as RC
from math import pi
from time import sleep
from RTIMU import RTIMU, Settings
from simple_pid import PID
altIMUSettings = Settings('AltIMU')
altIMU = RTIMU(altIMUSettings)
CC = RC()
print ('IMU Name:', altIMU.IMUName())


# Initialize the AltIMUv5
if not altIMU.IMUInit():
    raise Exception('IMU initialization failed!')


altIMU.setSlerpPower(0.02)
altIMU.setGyroEnable(True)
altIMU.setAccelEnable(True)
altIMU.setCompassEnable(True)

# Get minimum poll interval
poll_interval = altIMU.IMUGetPollInterval()
print(poll_interval)


# PID coefficients - still need to be tuned
KP = 20
KI = 0
KD = 0
startPoint = 1
#f= open("setting.txt","r")
pid = PID(KP,KI, KD, setpoint=startPoint)
pid.output_limits = (-127, 127)
pid.proportional_on_measurement = True
print(startPoint)

# Initialize some values
lastFusionRollX = 0.0
Ivalue = 0.0
orginal = 0;
orginalX = 0;
count = 1/0.02;
while True:
    #if (count == 1/0.02):
    #    count =0
    #    f= open("setting.txt","r")
    #    setting=f.read().split()
    #    startPoint = float(setting[0])
    #    KP=float(setting[1])
    #    KI=float(setting[2])
    #    KD=float(setting[3])
    #    pid.setpoint= startPoint 
    #    pid.Kp=KP
    #    pid.Ki=KI
    #    pid.Kd=KD
    
    # Check if AltIMU is ready
    if altIMU.IMURead():
        # Get a new set of data from IMU and pressure sensor
        data = altIMU.getIMUData()
        # Extract fused roll, pitch and yaw from the data (values are in radians)
        (fusionRollX, fusionPitchY, fusionYawZ) = data['fusionPose']
           # Calculate PID term
        fusionRollX = fusionRollX *180/pi
        print(fusionRollX)
        if orginal == 1:
        #    error = fusionRollX -orginalX
         #   Pvalue = KP * errors
          #  Ivalue += KI * error
           # Dvalue = KD * (error -  lastFusionRollX)
            #lastFusionRollX = error
            #PID = Pvalue + Ivalue + Dvalue
            # Calculate motor PWM. Divide PID by pi to normalize radians
            motorPWM = pid(fusionRollX)
           # print( motorPWM )
            CC.MotorDrive1(motorPWM,1)
            CC.MotorDrive2(motorPWM,1)
            # If integral part gets over-excited, limit to pi
            if Ivalue > pi:
                Ivalue = pi
            if Ivalue < -pi:
                Ivalue = -pi
            count = count +1
            sleep(0.02*0.001)
        else:
            orginal = 1
            
       
