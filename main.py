import cv2 as cv
from utils import *
from realsense import Camera
from lite6 import *

class Main:
    def __init__(self):
        self.arm = Manipulator('192.168.1.157')
        self.camera = Camera()
        self.utils_ = utils()
        self.arucodetecter = cv.aruco.ArucoDetector()
        self.camMatrix = self.camera.cameraMatrix()
        self.c2g = self.utils_.c2g(self.utils_.rotationMatrix,self.utils_.translationVector)

    def main(self):
        print(self.camMatrix)
        _,self.frame,_ = self.camera.get_frame_stream()
        self.gray = cv.cvtColor(self.frame, cv.COLOR_RGB2GRAY)
        self.corners, self.ids, self.rejectedImgPoints = self.arucodetecter.detectMarkers(self.gray)
        # print(corners,ids)
        self.c2t_affine  = self.utils_.getArucoPosition(1,self.camMatrix,self.ids,self.corners)
        print("aruco tvec ",self.c2t_affine[:3, 3])
        self.c2t = self.c2t_affine
        self.b2g = self.utils_.createAffine(*self.utils_.getrobotTransform(*(self.arm.get_position()[1])))
        print("b2g",self.b2g)
        self.b2c =  self.utils_.inverseAffine(self.c2g) @ self.b2g 
        print("b2c",self.b2c)
        self.result_position = self.c2t_affine @ self.b2c
        print("result_position",self.result_position)
        #self.arm.grabfromabove(self.result_position[:3, 3])
        # key = cv.waitKey(1)
        # if key == 27:
        # 	break
        self.camera.release()
        cv.destroyAllWindows()

if __name__ == '__main__':
    program = Main()
    program.main()
