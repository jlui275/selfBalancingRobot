
from math import pi
from time import sleep
from RTIMU import RTIMU, Settings
from RoboClawAdvance import RoboClawAdvance as RC
CC = RC()
CC.MotorDrive1(0,1)
CC.MotorDrive2(0,1)