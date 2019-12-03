from roboclaw import Roboclaw
from time import sleep
from roboclaw import Roboclaw
from time import sleep

class RoboClawAdvance:
    def __init__(self):
        self.PWM_MAX = 127
        self.address = 0x80
        self.roboclaw = Roboclaw("/dev/ttyS1", 38400)
        self.roboclaw.Open()
    def MotorDrive1(self,speed,direction):
        rpm = int(abs(speed)*direction)
        if speed>= 0:
            # Reverse
            self.roboclaw.ForwardM1(self.address,rpm)
        else:
            self.roboclaw.BackwardM1(self.address,rpm)
    def MotorDrive2(self,speed,direction):
        rpm = int(abs(speed)*direction)
        if speed >= 0:
            self.roboclaw.ForwardM2(self.address,rpm)
        else:
            # Forward / stoppe
            self.roboclaw.BackwardM2(self.address,rpm)

    

