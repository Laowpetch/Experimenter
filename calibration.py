import numpy as np
import os
import pickle as pkl
from realsense import Camera
from lite6 import Manipulator
from utils import *

class HandEyeCalibration:
    def __init__(self):
        # ! use self only for shared and const data
        self.filename = 'calib_data.pkl'
        if os.path.exists(self.filename):
            with open(self.filename, "rb") as file:
                self.c2g = pkl.load(file)
        else : 
            self.c2g = None
#         self.rotationMatrix = np.array([[ 0.15270852,  0.07282495 , 0.98558441],
#  [ 0.48849825,  0.86136814 ,-0.13933554],
#  [-0.85909811,  0.50273398 , 0.09596339]])

#         self.translationVector = np.array([[  76.93516569],
#  [  59.83190353],
#  [-172.75037391]])

    def calibrate(self):
        arm = Manipulator('192.168.1.157')
        camera = Camera()
        count  = 0
        row = 6
        column = 4
        pattern_size = [row , column]
        space = 10 # ! mm : Check for detail
        obj_points = np.array([])
        for y in range(column):
            for x in range(row):
                obj_points =np.append(obj_points,np.array([x*space,y*space,0]))
        # print("obj point",obj_points)
        obj_points = obj_points.reshape(row*column,3)

        camMatrix = camera.cameraMatrix()
        print("CamMatrix",camMatrix)
        distCoeff = np.array([0,0,0,0,0])

        ROTMAT_t2c = np.array([])
        TVEC_t2c = np.array([])
        ROTMAT_g2b = np.array([])
        TVEC_g2b = np.array([])
        keeploopalive = 1
        while (keeploopalive):
            frame = []
            while(1):
                _,frame,_ = camera.get_frame_stream()
                cv.imshow("windows",frame)
                k = cv.waitKey(50)
                if k == ord('s'):
                    print('Pressed : s')
                    break
                elif k == ord('q'):
                    print('Pressed : q')
                    keeploopalive = 0
                    break
            found ,img_points = cv.findCirclesGrid(frame, pattern_size ,flags=cv.CALIB_CB_SYMMETRIC_GRID +cv.CALIB_CB_CLUSTERING)
            if not found: 
                print("img point not found ")
                continue
            img_points = np.array(img_points)
            if len(img_points) != len(obj_points):
                print("not equal ")
                continue

            # * get target 2 cam
            _,rvec_c2t,tvec_c2t = cv.solvePnP(obj_points, img_points, camMatrix,distCoeff )
            aff_t2c = inverseAffine(createAffine(rvec_c2t, tvec_c2t))
            rotmat_t2c = aff_t2c[:3, :3]
            tvec_t2c = aff_t2c[:3, 3]
            print("t2c", rotmat_t2c,tvec_t2c) 
            
            # * get gripper 2 base
            rvec_g2b, tvec_g2b = getrobotTransform(*(arm.get_position()[1]))
            aff_g2b = inverseAffine(createAffine(rvec_g2b, tvec_g2b))
            rotmat_g2b = aff_g2b[:3, :3]
            tvec_g2b = aff_g2b[:3, 3]
            
            # * gather to RVEC TVEC
            ROTMAT_t2c = np.append(ROTMAT_t2c,rotmat_t2c)
            TVEC_t2c = np.append(TVEC_t2c,tvec_t2c)
            ROTMAT_g2b = np.append(ROTMAT_g2b,rotmat_g2b)
            TVEC_g2b = np.append(TVEC_g2b,tvec_g2b)
            
            count += 1
            print(count)
            print("successfully add")

        ROTMAT_t2c = ROTMAT_t2c.reshape(count,3,3)
        TVEC_t2c = TVEC_t2c.reshape(count,3,1)
        ROTMAT_g2b = ROTMAT_g2b.reshape(count,3,3)
        TVEC_g2b = TVEC_g2b.reshape(count,3,1)
        # cam 2 gripper 
        rotationMatrix, translationVector = cv.calibrateHandEye(ROTMAT_g2b,TVEC_g2b,ROTMAT_t2c,TVEC_t2c)
        self.c2g = createAffine(rotationMatrix, translationVector)
        print(self.c2g)
        key = input("Press 'y' to save: ")
        if key.lower() == 'y':
            with open(self.filename, "wb") as file:
                pkl.dump(self.c2g, file)
    
    def getcalibration(self) -> np.ndarray:
        if self.c2g is None:
            print("Please perform calibration first")
            exit()
        return self.c2g
    
if __name__ == '__main__':
    rMat,tvec = HandEyeCalibration().calibrate()
    print(rMat)
    print(tvec)