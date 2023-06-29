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
        self.distCoeff = np.array([0,0,0,0,0],np.float)
        
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
        RVEC_t2c = np.array([])
        TVEC_t2c = np.array([])
        RVEC_g2b = np.array([])
        TVEC_g2b = np.array([])
        count = len(img_points)
        for i in range(len(img_points)):
            # * get target 2 cam
            success,rvec_t2c,tvec_t2c = cv.solvePnP(self.obj_points, img_points[i], self.camMatrix,self.distCoeff )
            if not success:
                count -= 1
                continue
            # aff_t2c = createAffine(rvec_t2c, tvec_t2c)
            # rotmat_t2c = aff_t2c[:3, :3]
            # tvec_t2c = aff_t2c[:3, 3]
            # print("t2c", rvec_t2c,tvec_t2c) 
            
            # * get gripper 2 base
            rvec_g2b, tvec_g2b = getrobotTransform(*(arm_positions[i]))
            # aff_g2b = inverseAffine(createAffine(rvec_b2g, tvec_b2g))
            # rotmat_g2b = aff_g2b[:3, :3]
            # tvec_g2b = aff_g2b[:3, 3]
            
            # * gather to RVEC TVEC
            RVEC_t2c = np.append(RVEC_t2c,rvec_t2c)
            TVEC_t2c = np.append(TVEC_t2c,tvec_t2c)
            RVEC_g2b = np.append(RVEC_g2b,rvec_g2b)
            TVEC_g2b = np.append(TVEC_g2b,tvec_g2b)
            
        
        RVEC_t2c = RVEC_t2c.reshape(count,3,1)
        TVEC_t2c = TVEC_t2c.reshape(count,3,1)
        RVEC_g2b = RVEC_g2b.reshape(count,3,1)
        TVEC_g2b = TVEC_g2b.reshape(count,3,1)

        # ####
        # R_base2gripper = []
        # t_base2gripper = []
        # for Ro, t in zip(RVEC_g2b , TVEC_g2b):
        #     print(Ro)
        #     R_b2g = np.transpose(Ro)
        #     t_b2g = -R_b2g @ t
        #     R_base2gripper.append(R_b2g)
        #     t_base2gripper.append(t_b2g)

        # RVEC_g2b = R_base2gripper
        # TVEC_g2b = t_base2gripper
        # ####

        
        # cam 2 gripper 
        rotationMatrix, translationVector = cv.calibrateHandEye(RVEC_g2b,TVEC_g2b,RVEC_t2c,TVEC_t2c,method =3)
        print("calibrate methods result")
        a, b = cv.calibrateHandEye(RVEC_g2b,TVEC_g2b,RVEC_t2c,TVEC_t2c,method =0)
        print("method0",a, b)
        print("degrees",np.degrees(rotation_matrix_to_euler_angles(a)))
        a, b = cv.calibrateHandEye(RVEC_g2b,TVEC_g2b,RVEC_t2c,TVEC_t2c,method =1)
        print("method1",a, b)
        print("degrees",np.degrees(rotation_matrix_to_euler_angles(a)))
        a, b = cv.calibrateHandEye(RVEC_g2b,TVEC_g2b,RVEC_t2c,TVEC_t2c,method =2)
        print("method2",a, b)
        print("degrees",np.degrees(rotation_matrix_to_euler_angles(a)))
        a, b = cv.calibrateHandEye(RVEC_g2b,TVEC_g2b,RVEC_t2c,TVEC_t2c,method =3)
        print("method3",a, b)
        print("degrees",np.degrees(rotation_matrix_to_euler_angles(a)))
        a, b = cv.calibrateHandEye(RVEC_g2b,TVEC_g2b,RVEC_t2c,TVEC_t2c,method =4)
        print("method4",a, b)
        print("degrees",np.degrees(rotation_matrix_to_euler_angles(a)))
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
            print("armpose",self.arm.get_position())
            print("takingimage")
            frame = ''
            cv.waitKey(1000)
            time.sleep(1)
            _,frame,_ = self.camera.get_frame_stream()
            frame_copy = frame.copy()
                # if ord('s')== cv.waitKey(-1):
                #     break
            found ,img_points = cv.findCirclesGrid(frame, self.pattern_size ,flags=cv.CALIB_CB_SYMMETRIC_GRID )
            cv.drawChessboardCorners(frame_copy , self.pattern_size, img_points,found)
            # success,rvec,tvec = cv.solvePnP(self.obj_points, img_points, self.camMatrix,self.distCoeff)
            # cv.drawFrameAxes( frame_copy, self.camMatrix,self.distCoeff, rvec,tvec, length=10 ) 

            cv.imshow("window",frame_copy)
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
            frame_copy =[]
            found = 0
            img_points = []
            while(1):
                _,frame,_ = self.camera.get_frame_stream()
                frame_copy = frame.copy()
                found ,img_points = cv.findCirclesGrid(frame, self.pattern_size ,flags=cv.CALIB_CB_SYMMETRIC_GRID )
                cv.drawChessboardCorners(frame_copy , self.pattern_size, img_points,found)
                if found:
                    success,rvec,tvec = cv.solvePnP(self.obj_points, img_points, self.camMatrix,self.distCoeff)
                    cv.drawFrameAxes( frame_copy, self.camMatrix,self.distCoeff, rvec,tvec, length=10 ) 
                cv.imshow("windows",frame_copy)
                k = cv.waitKey(50)
                if k == ord('s'):
                    cv.imwrite("static_images/"+str(0)+'.png', frame)
                    print('Pressed : s')
                    break
                elif k == ord('q'):
                    print('Pressed : q')
                    keeploopalive = 0
                    break
            
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
        print("Press y to save calibration")
        key = cv.waitKey(-1)
        if key == ord('y'):
            with open(self.filename, "wb") as file:
                pkl.dump(self.c2g, file)
            print("save")
    
    def static_calibration(self) :
        self.initcalibrate()
        cv.namedWindow("window")
        count = 0
        image_points_list = []
        arm_positions_list = []
        
        self.arm.arm.set_position(x=120, y=330, z=300, roll=180, pitch=0, yaw=90, speed=50, wait=True)
        pose = self.arm.get_position()[1]


        frame=cv.imread("static_images/"+str(0)+'.png')
        
            # if ord('s')== cv.waitKey(-1):
            #     break
        found ,img_points = cv.findCirclesGrid(frame, self.pattern_size ,flags=cv.CALIB_CB_SYMMETRIC_GRID )
        cv.drawChessboardCorners(frame, self.pattern_size, img_points,found)
        if not found: 
            print("img point not found ")
            exit(1)
        img_points = np.array(img_points)

        rows  = 15
        cols = 10
        ideal_obj_points = []
        ideal_corners = np.array([[-2325,2516,823],
                                    [-2318,2656,823],
                                    [-2236,2511,824],
                                    [-2228,2651,824]])
        top_left, top_right,bottom_left,bottom_right  = ideal_corners
        column_spacing = (bottom_left - top_left) / (cols - 1)
        row_spacing = (top_right - top_left) / (rows - 1)
        for i in range(cols):
            for j in range(rows):
                point = top_left + (i * column_spacing) + (j * row_spacing)
                ideal_obj_points.append(point)
        ideal_obj_points = np.array(ideal_obj_points)
        if len(img_points) != len(ideal_obj_points):
            print("not equal ")
            exit(1)
        
        success , rvec_t2c ,tvec_t2c = cv.solvePnP(ideal_obj_points,img_points,self.camMatrix,self.distCoeff)
        cv.drawFrameAxes( frame, self.camMatrix,self.distCoeff, rvec_t2c, top_left.reshape(3,1)-tvec_t2c, length=100 ) 
        if not success :
            exit(1)
        # rotation axis
        axis = np.array([0, 0, 0])
        # rotaion degree (90degree to radian)
        angle = np.radians(-90)
        # calc rvec
        rvec_t2g = axis * angle
        print("rvec_t2g",rvec_t2g)
        tvec_t2g = np.array([-2207,2556,1029])
        
        g2t = inverseAffine(createAffine(rvec_t2g.reshape(3,1),tvec_t2g.reshape(3,1)))
        t2c = createAffine(rvec_t2c,tvec_t2c)
        g2c = g2t @ t2c 
        self.c2g = inverseAffine(g2c)
        
    # with open('images/img_position.pkl', "wb") as file:
    #     pkl.dump(image_points_list, file)
    # with open('images/arm_position.pkl', "wb") as file:
    #     pkl.dump(arm_positions_list, file)
        cv.imshow("window",frame)
        
        print(self.c2g)
        print('eular angle')
        print(np.degrees(rotation_matrix_to_euler_angles(self.c2g)))
        cv.displayOverlay("window","press y to save to file")
        
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
    # calibration.calibrate() 
    # calibration.static_calibration() 
    # c2g = calibration.getcalibration()
    # print(c2g)
