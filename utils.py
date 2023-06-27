import cv2 as cv 
import numpy as np
import math

def euler_to_rotmat(roll, pitch, yaw) -> np.ndarray:
    """convert Eular Angle in degrees to Rotation Matrix

    Args:
        roll (double): radian
        pitch (double): radian
        yaw (double): radian

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

    rotation_matrix = rotation_x.dot(rotation_y).dot(rotation_z)

    return rotation_matrix 

def rotation_matrix_to_euler_angles(R):
    yaw = math.atan2(R[2, 1], R[2, 2])
    pitch = math.atan2(-R[2, 0], math.sqrt(R[2, 1]**2 + R[2, 2]**2))
    roll = math.atan2(R[1, 0], R[0, 0])

    return roll, pitch, yaw

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
    
if __name__ == '__main__':
    rotationMatrix = np.array([[ 0.15270852,  0.07282495 , 0.98558441],
 [ 0.48849825,  0.86136814 ,-0.13933554],
 [-0.85909811,  0.50273398 , 0.09596339]])
    print(rotationMatrix)
    a = rotation_matrix_to_euler_angles(rotationMatrix)
    print(np.degrees(a))
    b = euler_to_rotmat(*a)
    print(b)

#         self.translationVector = np.array([[  76.93516569],
#  [  59.83190353],
#  [-172.75037391]])
