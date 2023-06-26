import time

from xarm.wrapper import XArmAPI

ip = '192.168.1.157'

arm = XArmAPI(ip)

class Lite6Interface():
    def __init__(self):
        arm.motion_enable(enable=True)
        arm.set_gripper_enable(True)
        arm.set_mode(0)
        arm.set_state(state=0)

    def test(self):
        arm.reset(wait=True)
        arm.set_tool_position(x=100, y=0, z=0, roll=0, pitch=0, yaw=0, speed=50, wait=True)
        print(arm.get_position(), arm.get_position(is_radian=True))
        arm.set_tool_position(x=0, y=200, z=0, roll=0, pitch=0, yaw=0, speed=200, wait=True)
        print(arm.get_position(), arm.get_position(is_radian=True))
        arm.set_tool_position(x=200, y=0, z=0, roll=0, pitch=0, yaw=0, speed=300, wait=True)
        print(arm.get_position(), arm.get_position(is_radian=True))
        arm.set_tool_position(x=0, y=-400, z=0, roll=0, pitch=0, yaw=0, speed=500, wait=True)
        print(arm.get_position(), arm.get_position(is_radian=True))
        arm.set_tool_position(x=-200, y=0, z=0, roll=0, pitch=0, yaw=0, speed=500, wait=True)
        print(arm.get_position(), arm.get_position(is_radian=True))
        arm.set_tool_position(x=0, y=200, z=0, roll=0, pitch=0, yaw=0, speed=600, wait=True)
        print(arm.get_position(), arm.get_position(is_radian=True))
        arm.reset(wait=True)
        arm.disconnect()

    def pouring_1(self):
        self.ready()
        time.sleep(5)
        arm.set_position(x=110.9, y=282.9, z=16.9, roll=-104.6, pitch=-87, yaw=13.2, speed=50, wait=True)
        arm.set_position(x=110.9, y=400, z=16.9, roll=-104.6, pitch=-87, yaw=13.2, speed=50, wait=True)

        #pick
        #arm.close_lite6_gripper()
        time.sleep(5)

        arm.set_position(x=110.9, y=400, z=74.8, roll=-104.6, pitch=-87, yaw=7.7, speed=50, wait=True)
        arm.set_position(x=110.9, y=282.9, z=74.8, roll=-104.6, pitch=-87, yaw=7.7, speed=50, wait=True)

        arm.set_position(x=-31.1, y=282.9, z=74.8, roll=-15.1, pitch=-82.2, yaw=13.2, speed=50, wait=True)
        arm.set_position(x=-31.1, y=372.2, z=74.8, roll=-15.1, pitch=-82.2, yaw=13.2, speed=50, wait=True)

        #pour
        arm.set_position(x=-31.1, y=372.2, z=74.8, roll=-87, pitch=13.3, yaw=89.7, speed=50, wait=True)
        time.sleep(10)

        arm.set_position(x=-31.1, y=372.2, z=74.8, roll=-15.1, pitch=-82.2, yaw=13.2, speed=50, wait=True)
        arm.set_position(x=-31.1, y=282.9, z=74.8, roll=-15.1, pitch=-82.2, yaw=13.2, speed=50, wait=True)
    
        arm.set_position(x=110.9, y=282.9, z=74.8, roll=-104.6, pitch=-87, yaw=7.7, speed=50, wait=True)
        arm.set_position(x=110.9, y=400, z=74.8, roll=-104.6, pitch=-87, yaw=13.2, speed=50, wait=True)
        arm.set_position(x=110.9, y=400, z=16.9, roll=-104.6, pitch=-87, yaw=13.2, speed=50, wait=True)

        #place
        #arm.open_lite6_gripper()
        #arm.stop_lite6_gripper()
        time.sleep(2)

        arm.set_position(x=110.9, y=282.9, z=16.9, roll=-104.6, pitch=-87, yaw=13.2, speed=50, wait=True)
        
        arm.reset()

    def testInput(self):
        code, digitals = arm.get_cgpio_digital()
        print('get_cgpio_digital, code={}, digitals={}'.format(code, digitals))

    def ready(self):
        arm.set_position(x=200,y=0,z=200.1,roll=180,pitch=0,yaw=0,speed=50,wait=True)
    
    def reset(self):
        arm.reset(speed=50)

def main(args=None):
    node = Lite6Interface()
    node.ready()
    node.reset()

if __name__ == '__main__':
    main()