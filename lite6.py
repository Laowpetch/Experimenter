from xarm.wrapper import XArmAPI
import time
class Manipulator():
    def __init__(self,ip):
        self.arm = XArmAPI(ip)
        self.arm.motion_enable(enable=True)
        self.arm.set_gripper_enable(True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)

    def grabfromabove(self,objectposition):
        print('grabing')
        self.arm.set_position(objectposition[0],objectposition[1],objectposition[2]+150,roll=180,pitch=0,yaw=90,speed=50,wait=True)
        
    def set_position(self,x, y ,z,roll , pitch ,yaw):
        self.arm.set_position(x, y ,z,roll , pitch ,yaw,speed=30,wait=True)
        # time.sleep(5)
        
    def get_position(self):
        return self.arm.get_position()

    def ready(self):
        self.arm.set_position(x=200,y=0,z=200.1,roll=180,pitch=0,yaw=0,speed=50,wait=True)
    
    def reset(self):
        self.arm.reset(speed=50)