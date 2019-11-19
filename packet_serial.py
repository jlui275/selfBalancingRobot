from roboclaw import Roboclaw
from time import sleep

if __name__ == "__main__":
    
    address = 0x80
    roboclaw = Roboclaw("/dev/ttyS1", 9600)
    roboclaw.Open()
    
    while True:
        
     
        
        roboclaw.ForwardM1(address,127)
        sleep(2)
        roboclaw.ForwardM1(address,0)
        sleep(2)
        roboclaw.ForwardM1(address,-1)
        sleep(2)
        
        
        
    
    

