from roboclaw import Roboclaw
from time import sleep
from roboclaw import Roboclaw
from time import sleep

class RoboClawAdvance:
   def __init__(self):
        self.PWM_MAX = 127
        self.address = 0x80
        self.roboclaw = Roboclaw("/dev/ttyS1", 9600)
        self.roboclaw.Open()
    def MotorDrive1(self,speed);
        if speed < 0:
            # Reverse
            pwm = int(PWM_MAX * speed)
            if pwm > PWM_MAX:
                pwm = PWM_MAX
            roboclaw.BackwardM1(address,pwm)
        else:
            # Forward / stoppe
            pwm = int(PWM_MAX * speed)
            if pwm > PWM_MAX:
                pwm = PWM_MAX
            roboclaw.ForwardM1(address,pwm)
    def MotorDrive2(self,speed):
          if speed < 0:
            # Reverse
            pwm = int(PWM_MAX * speed)
            if pwm > PWM_MAX:
                pwm = PWM_MAX
            roboclaw.BackwardM2(address,pwm)
        else:
            # Forward / stoppe
            pwm = int(PWM_MAX * speed)
            if pwm > PWM_MAX:
                pwm = PWM_MAX
            roboclaw.ForwardM2(address,pwm)
if __name__ == "__main__":
    address = 0x80
    roboclaw = Roboclaw("/dev/ttyS1", 9600)
    roboclaw.Open()
    
    while True:
       
        sleep(2)
        roboclaw.ForwardM1(address,0)
        roboclaw.ForwardM(address,0)
        sleep(2)
        
        
        
    
    


    

