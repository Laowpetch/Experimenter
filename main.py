import cv2 as cv
from utils import *
from realsense import Camera
from lite6 import *
from calibration import *

if __name__ == '__main__':     
    c2g = HandEyeCalibration().getcalibration()
    arm = Manipulator('192.168.1.157')
    camera = Camera()
    arucodetecter = cv.aruco.ArucoDetector()
    camMatrix = camera.cameraMatrix()
    print(camMatrix)
    
    cv.namedWindow("window")
    # print("testing")
    
    while 1:
        _,frame,_ = camera.get_frame_stream()
        gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
        corners, ids, _ = arucodetecter.detectMarkers(gray)
        
        # print(corners,ids)
        if ids is not None:
            if len(ids)>0:
                t2c  = getArucoPosition(1,camMatrix,ids,corners)
                if t2c is not None:
                    # print("aruco tvec ",self.c2t_affine[:3, 3])
                    c2t = inverseAffine(t2c)
                    c2t = c2t[:, 3]
                    
                    b2g = createAffine(*getrobotTransform(*(arm.get_position()[1])))
                    # print("b2g",self.b2g)
                    g2b = inverseAffine(b2g)
                    g2c = inverseAffine(c2g) 
                    # print("b2c",self.b2c)
                    print("g2t",g2c @ c2t)
                    result_position =  b2g @ (g2c @ c2t)
                    cv.displayOverlay("window",str(result_position))                 
                    print("result_position",result_position)
                    
                    # self.arm.grabfromabove(self.result_position[:3, 3])
                frame = cv.aruco.drawDetectedMarkers(frame, corners,ids)         
        cv.imshow("window",frame)
        key = cv.waitKey(10)
        if key == ord('q'):
            break
    camera.release()
    cv.destroyAllWindows()
    
# # ! this is no need 
# if __name__ == '__main__':
#     program = Main()
#     program.main()
