import cv2 as cv 
import numpy as np
import math

def euler_to_rotmat(roll, pitch, yaw) -> np.ndarray:
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

    rotation_matrix = rotation_z.dot(rotation_y).dot(rotation_x)

    return rotation_matrix      
def getrobotTransform(x, y ,z,roll , pitch ,yaw) -> tuple[np.ndarray,np.ndarray]:
    
    # gripper rotation
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    rotation_matrix = euler_to_rotmat(roll, pitch, yaw)
    rvec, _ = cv.Rodrigues(rotation_matrix)
    
    # gripper 2 base
    rvec_b2g = np.array(rvec).reshape(3,1)
    tvec_b2g = np.array([x,y,z]).reshape(3,1)
    
    return rvec_b2g, tvec_b2g
    
def createAffine(r : np.ndarray, tvec : np.ndarray) -> np.ndarray:
    R = []
    if r.shape == (3,1):
        R ,_  = cv.Rodrigues(r)
    elif r.shape == (3,3) :
        R = r
    affine_matrix = np.eye(4)
    affine_matrix[:3, :3] = R
    affine_matrix[:3, 3] = tvec.reshape(3)
    return affine_matrix

def inverseAffine(affine_matrix:np.ndarray):
    R = affine_matrix[:3, :3]
    tvec = affine_matrix[:3, 3]
    inv_R = np.transpose(R)
    inv_t = - inv_R @ tvec
    return createAffine(inv_R,inv_t)

def getArucoPosition( num_id,camera_matrix, ids, corners):
    size = 24
    obj_points = []
    obj_points =np.append(obj_points,np.array([0,0,0]))
    obj_points =np.append(obj_points,np.array([size,0,0]))
    obj_points =np.append(obj_points,np.array([size,size,0]))
    obj_points =np.append(obj_points,np.array([0,size,0]))
    obj_points = obj_points.reshape(4,3)
    if num_id in np.ravel(ids) :
        _, rvec, tvec = cv.solvePnP(obj_points,corners[num_id-1][0], camera_matrix, np.array([0,0,0,0,0]))
        return createAffine(rvec,tvec)
    else :
        return None
    
# ! this func is no need
# def c2g(self,rotationMatrix,TranslationVector):
#     return self.createAffine(rotationMatrix,TranslationVector)
