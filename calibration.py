import numpy as np
import os
import pickle as pkl
import cv2 as cv
from realsense import Camera
from lite6 import Manipulator
from utils import *
import time

class HandEyeCalibration:
    def __init__(self):
        # ! use self only for shared and const data
        self.filename = 'calib_data.pkl'
        if os.path.exists(self.filename):
            with open(self.filename, "rb") as file:
                self.c2g = pkl.load(file)
        else : 
            self.c2g = None
            
    def initcalibrate(self):
        self.arm = Manipulator('192.168.1.157')
        self.camera = Camera()
        row = 15
        column = 10
        self.pattern_size = [row , column]
        space = 10 # ! mm : Check for detail
        self.obj_points = np.array([])
        for y in range(column):
            for x in range(row):
                self.obj_points =np.append(self.obj_points,np.array([x*space,y*space,0]))
        # print("obj point",obj_points)
        self.obj_points = self.obj_points.reshape(row*column,3)

        self.camMatrix = self.camera.cameraMatrix()
        print("CamMatrix",self.camMatrix)
        self.distCoeff = np.array([0,0,0,0,0])
        
    def recordpattern(self):
        self.initcalibrate()
        positions = []
        count = 1
        while (1):
            _,frame,_ = self.camera.get_frame_stream()
            cv.imshow("windows",frame)
            k = cv.waitKey(50)
            if k == ord('s'):
                print('Pressed : s , count = ',count)
                count+=1
                positions.append(self.arm.get_position()[1])
                
            elif k == ord('q'):
                print('Pressed : q')
                with open('calib_position.pkl', "wb") as file:
                    pkl.dump(positions, file)
                break
    def calculate_c2g(self,  img_points, arm_positions) -> np.ndarray:
        if len(img_points) != len(arm_positions):
            print('somethingwrong')
            exit(1)
        ROTMAT_t2c = np.array([])
        TVEC_t2c = np.array([])
        ROTMAT_g2b = np.array([])
        TVEC_g2b = np.array([])
        for i in range(len(img_points)):
            # * get target 2 cam
            _,rvec_t2c,tvec_t2c = cv.solvePnP(self.obj_points, img_points[i], self.camMatrix,self.distCoeff )
            # aff_t2c = createAffine(rvec_t2c, tvec_t2c)
            # rotmat_t2c = aff_t2c[:3, :3]
            # tvec_t2c = aff_t2c[:3, 3]
            print("t2c", rvec_t2c,tvec_t2c) 
            
            # * get gripper 2 base
            rvec_b2g, tvec_b2g = getrobotTransform(*(arm_positions[i]))
            aff_g2b = inverseAffine(createAffine(rvec_b2g, tvec_b2g))
            rotmat_g2b = aff_g2b[:3, :3]
            tvec_g2b = aff_g2b[:3, 3]
            
            # * gather to RVEC TVEC
            ROTMAT_t2c = np.append(ROTMAT_t2c,rvec_t2c)
            TVEC_t2c = np.append(TVEC_t2c,tvec_t2c)
            ROTMAT_g2b = np.append(ROTMAT_g2b,rotmat_g2b)
            TVEC_g2b = np.append(TVEC_g2b,tvec_g2b)
            
        count = len(img_points)
        ROTMAT_t2c = ROTMAT_t2c.reshape(count,3,1)
        TVEC_t2c = TVEC_t2c.reshape(count,3,1)
        ROTMAT_g2b = ROTMAT_g2b.reshape(count,3,3)
        TVEC_g2b = TVEC_g2b.reshape(count,3,1)
        # cam 2 gripper 
        rotationMatrix, translationVector = cv.calibrateHandEye(ROTMAT_g2b,TVEC_g2b,ROTMAT_t2c,TVEC_t2c,method =cv.CALIB_HAND_EYE_DANIILIDIS)
        print("calibrate methods result")
        a, b = cv.calibrateHandEye(ROTMAT_g2b,TVEC_g2b,ROTMAT_t2c,TVEC_t2c,method =0)
        print(a, b)
        print(np.degrees(rotation_matrix_to_euler_angles(a)))
        a, b = cv.calibrateHandEye(ROTMAT_g2b,TVEC_g2b,ROTMAT_t2c,TVEC_t2c,method =1)
        print(a, b)
        print(np.degrees(rotation_matrix_to_euler_angles(a)))
        a, b = cv.calibrateHandEye(ROTMAT_g2b,TVEC_g2b,ROTMAT_t2c,TVEC_t2c,method =2)
        print(a, b)
        print(np.degrees(rotation_matrix_to_euler_angles(a)))
        a, b = cv.calibrateHandEye(ROTMAT_g2b,TVEC_g2b,ROTMAT_t2c,TVEC_t2c,method =3)
        print(a, b)
        print(np.degrees(rotation_matrix_to_euler_angles(a)))
        a, b = cv.calibrateHandEye(ROTMAT_g2b,TVEC_g2b,ROTMAT_t2c,TVEC_t2c,method =4)
        print(a, b)
        print(np.degrees(rotation_matrix_to_euler_angles(a)))
        print("------------------")
        return createAffine(rotationMatrix, translationVector)
    
    def autocalibrate(self):
        self.initcalibrate()
        cv.namedWindow("window")
        if os.path.exists('calib_position.pkl'):
            with open('calib_position.pkl', "rb") as file:
                self.positions = pkl.load(file)
        else : 
            print("no pose found ")
            exit()
        count = 0
        image_points_list = []
        arm_positions_list = []
        for pose in self.positions:
            count +=1
            self.arm.set_position(*pose)
            print("takingimage")
            frame = ''
            cv.waitKey(1000)
            time.sleep(1)
            _,frame,_ = self.camera.get_frame_stream()
                # cv.imshow("window",frame)
                # if ord('s')== cv.waitKey(-1):
                #     break
            found ,img_points = cv.findCirclesGrid(frame, self.pattern_size ,flags=cv.CALIB_CB_SYMMETRIC_GRID +cv.CALIB_CB_CLUSTERING)
            if not found: 
                print("img point not found ")
                continue
            img_points = np.array(img_points)
            if len(img_points) != len(self.obj_points):
                print("not equal ")
                continue
            
            cv.imwrite("images/"+str(count)+'.png', frame)
            image_points_list.append(img_points)
            arm_positions_list.append(pose)
        with open('images/img_position.pkl', "wb") as file:
            pkl.dump(image_points_list, file)
        with open('images/arm_position.pkl', "wb") as file:
                pkl.dump(arm_positions_list, file)
        
        self.c2g = self.calculate_c2g(image_points_list, arm_positions_list)
        print(self.c2g)
        print('eular angle')
        print(np.degrees(rotation_matrix_to_euler_angles(self.c2g)))
        cv.displayOverlay("window","press y to save to file")
        key = cv.waitKey(-1)
        if key == ord('y'):
            with open(self.filename, "wb") as file:
                pkl.dump(self.c2g, file)
        
            
    def calibrate(self) :
        self.initcalibrate()
        keeploopalive = 1
        image_points_list = []
        arm_positions_list = []
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
                found ,img_points = cv.findCirclesGrid(frame, self.pattern_size ,flags=cv.CALIB_CB_SYMMETRIC_GRID +cv.CALIB_CB_CLUSTERING)
                if not found: 
                    print("img point not found ")
                    continue
                img_points = np.array(img_points)
                if len(img_points) != len(self.obj_points):
                    print("not equal ")
                    continue
                image_points_list.append(img_points)
                arm_positions_list.append(self.arm.get_position()[1])
            
        self.c2g = self.calculate_c2g(image_points_list, arm_positions_list)
        print(self.c2g)
        key = cv.waitKey(-1)
        if key == ord('y'):
            with open(self.filename, "wb") as file:
                pkl.dump(self.c2g, file)
    
    def getcalibration(self) -> np.ndarray:
        if self.c2g is None:
            print("Please perform calibration first")
            exit()
        return self.c2g
    
if __name__ == '__main__':
    calibration = HandEyeCalibration()
    #calibration.recordpattern()
    calibration.autocalibrate() 
    # c2g = calibration.getcalibration()
    # print(c2g)
