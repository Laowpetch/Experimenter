import cv2 as cv 
import numpy as np
import math

from realsense import Camera
from lite6 import Manipulator

class utils:
    def __init__(self):
        self.rotationMatrix = np.array([[ 0.15270852,  0.07282495 , 0.98558441],
 [ 0.48849825,  0.86136814 ,-0.13933554],
 [-0.85909811,  0.50273398 , 0.09596339]])

        self.translationVector = np.array([[  76.93516569],
 [  59.83190353],
 [-172.75037391]])

    def calibration(self):
        arm = Manipulator('192.168.1.157')
        camera = Camera()
        count  = 0
        row = 6
        column = 4
        pattern_size = [row , column]
        space = 10
        obj_points = np.array([])
        for y in range(column):
            for x in range(row):
                obj_points =np.append(obj_points,np.array([x*space,y*space,0]))
        # print("obj point",obj_points)
        obj_points = obj_points.reshape(row*column,3)

        camMatrix = camera.cameraMatrix()
        print("CamMatrix",camMatrix)
        distCoeff = np.array([0,0,0,0,0])

        RVEC_t2c = np.array([])
        TVEC_t2c = np.array([])
        RVEC_g2b = np.array([])
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
                    count+=1
                    break
                elif k == ord('q'):
                    print('Pressed : q')
                    keeploopalive = 0
                    count+=1
                    break
            found ,img_points = cv.findCirclesGrid(frame, pattern_size ,flags=cv.CALIB_CB_SYMMETRIC_GRID +cv.CALIB_CB_CLUSTERING)
            if not found: 
                print("img point not found ")
                count-=1
                continue
            img_points = np.array(img_points)
            if len(img_points) != len(obj_points):
                count-=1
                print("not equal ")
                continue

            # print("len point",len(img_points), " ", len(obj_points))
            # print(img_points)
            # * target 2 cam
            _,rvec_c2t,tvec_c2t = cv.solvePnP(obj_points, img_points, camMatrix,distCoeff )
            aff_t2c = self.inverseAffine(self.createAffine(rvec_c2t, tvec_c2t))
            rvec_t2c = aff_t2c[:3, :3]
            tvec_t2c = aff_t2c[:3, 3]
            print("t2c", rvec_t2c,tvec_t2c) 
            # print("t2c",rvec_t2c,tvec_t2c)
            # * get robot position
            _ , armposition = arm.get_position()
            rvec_g2b, tvec_g2b = self.getrobotTransform(*armposition)
            aff_g2b = self.inverseAffine(self.createAffine(rvec_g2b, tvec_g2b))
            rvec_g2b = aff_g2b[:3, :3]
            tvec_g2b = aff_g2b[:3, 3]
            # * gather to RVEC TVEC
            RVEC_t2c = np.append(RVEC_t2c,rvec_t2c)
            TVEC_t2c = np.append(TVEC_t2c,tvec_t2c)
            RVEC_g2b = np.append(RVEC_g2b,rvec_g2b)
            TVEC_g2b = np.append(TVEC_g2b,tvec_g2b)
            
            print(count)
            print("successfully add")

        RVEC_t2c = RVEC_t2c.reshape(count,3,3)
        TVEC_t2c = TVEC_t2c.reshape(count,3,1)
        RVEC_g2b = RVEC_g2b.reshape(count,3,3)
        TVEC_g2b = TVEC_g2b.reshape(count,3,1)
        # cam 2 gripper 
        rvec_c2g , tvec_c2g = cv.calibrateHandEye(RVEC_g2b,TVEC_g2b,RVEC_t2c,TVEC_t2c)
        print(rvec_c2g, tvec_c2g)
        return rvec_c2g, tvec_c2g
    
    def euler_to_rotmat(self,roll, pitch, yaw) -> np.ndarray:
        """convert Eular Angle in degrees to Rotation Matrix

        Args:
            roll (double): _description_
            pitch (double): _description_
            yaw (double): _description_

        Returns:
            np.ndarray: 3x3 rotation matrix
        """        
        rotation_x = np.array([[1, 0, 0],
                            [0, np.cos(yaw), -np.sin(yaw)],
                            [0, np.sin(yaw), np.cos(yaw)]])

        rotation_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                            [0, 1, 0],
                            [-np.sin(pitch), 0, np.cos(pitch)]])

        rotation_z = np.array([[np.cos(roll), -np.sin(roll), 0],
                            [np.sin(roll), np.cos(roll), 0],
                            [0, 0, 1]])

        # Compute the combined rotation matrix
        rotation_matrix = rotation_z.dot(rotation_y).dot(rotation_x)

        return rotation_matrix      
    def getrobotTransform(self,x, y ,z,roll , pitch ,yaw) -> tuple[np.ndarray,np.ndarray]:
            # gripper rotation
            roll = math.radians(roll)
            pitch = math.radians(pitch)
            yaw = math.radians(yaw)
            rotation_matrix = self.euler_to_rotmat(roll, pitch, yaw)
            rvec, _ = cv.Rodrigues(rotation_matrix)
            
            # gripper 2 base
            rvec_g2b = np.array(rvec).reshape(3,1)
            # print("rvecg2b" ,rvec_g2b )
            tvec_g2b = np.array([x,y,z]).reshape(3,1)
            # print("tvecg2b" ,tvec_g2b )
            # print("g2b",rvec_g2b.shape,tvec_g2b.shape)
            
            return rvec_g2b, tvec_g2b
        
    def createAffine(self, r : np.ndarray, tvec : np.ndarray) -> np.ndarray:
        R = []
        if r.shape == (3,1):
            R ,_  = cv.Rodrigues(r)
        elif r.shape == (3,3) :
            R = r
        affine_matrix = np.eye(4)
        affine_matrix[:3, :3] = R
        affine_matrix[:3, 3] = tvec.reshape(3)
        return affine_matrix

    def inverseAffine(self, affine_matrix:np.ndarray):
        R = affine_matrix[:3, :3]
        tvec = affine_matrix[:3, 3]
        inv_R = np.transpose(R)
        inv_t = - inv_R @ tvec
        return self.createAffine(inv_R,inv_t)

    def getArucoPosition(self, num_id,camera_matrix, ids, corners):
        size = 24
        obj_points = []
        obj_points =np.append(obj_points,np.array([0,0,0]))
        obj_points =np.append(obj_points,np.array([24,0,0]))
        obj_points =np.append(obj_points,np.array([24,24,0]))
        obj_points =np.append(obj_points,np.array([0,24,0]))
        obj_points = obj_points.reshape(4,3)
        if num_id in np.ravel(ids) :
            # print(len(obj_points),len(corners[num_id-1][0]))
            _, rvec, tvec = cv.solvePnP(obj_points,corners[num_id-1][0], camera_matrix, np.array([0,0,0,0,0]))
            return self.createAffine(rvec,tvec)
        else :
            return None
    
    def c2g(self,rotationMatrix,TranslationVector):
        return self.createAffine(rotationMatrix,TranslationVector)

if __name__ == "__main__":
    utils().calibration()
    # test = utils()
    # aff = test.createAffine(test.rotationMatrix,test.translationVector)
    # a = (test.inverseAffine(aff))
    # b = test.inverseAffine(a)
    # print(aff)
    # print(a)
    # print(b)