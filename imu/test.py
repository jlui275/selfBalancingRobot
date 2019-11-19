from altimu import AltIMU
from time import sleep
from datetime import datetime

imu = AltIMU()
imu.enable()

imu.calibrateGyroAngles()

start = datetime.now()

count = 0
while count < 100:
    stop = datetime.now() - start
    start = datetime.now()
    deltaT = stop.microseconds/1000000.0
    
    print " "
    print "Loop:", deltaT
    print "Accel:", imu.getAccelerometerAngles()
    print "Gyro:", imu.trackGyroAngles(deltaT = deltaT)
    print "Kalman:", imu.getKalmanAngles(deltaT = deltaT)
    count += 1;
    sleep(0.02)
    