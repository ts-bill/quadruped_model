import numpy as np

def Rotation_X(roll):
    """ Rotation matrix arround x (roll)
    """
#    roll = np.radians(roll)
    return np.matrix([[1,            0,             0, 0],
                      [0, np.cos(roll), -np.sin(roll), 0],
                      [0, np.sin(roll),  np.cos(roll), 0],
                      [0,            0,             0, 1]])

def Rotation_Y(pitch):
    """ Rotation matrix arround y (pitch)
    """
#    pitch = np.radians(pitch)
    return np.matrix([[ np.cos(pitch), 0, np.sin(pitch), 0],
                      [             0, 1,             0, 0],
                      [-np.sin(pitch), 0, np.cos(pitch), 0],
                      [             0, 0,             0, 1]])

def Rotation_Z(yaw):
    """ Rotation matrix arround z (yaw)
    """
#    yaw = np.radians(yaw)
    return np.matrix([[np.cos(yaw), -np.sin(yaw), 0, 0],
                      [np.sin(yaw),  np.cos(yaw), 0, 0],
                      [          0,            0, 1, 0],
                      [          0,            0, 0, 1]])
    
def Rotation_XYZ(roll , pitch , yaw):
    if roll != 0. or pitch != 0. or yaw != 0.:
        R = Rotation_X(roll)*Rotation_Y(pitch)*Rotation_Z(yaw)
        return R
    else:
        return np.identity(4)
    

def Tranformationmatrix(orientation, position):
    """compose translation and rotation"""
    roll = orientation[0]
    pitch = orientation[1]
    yaw = orientation[2]
    x0 = position[0]
    y0 = position[1]
    z0 = position[2]
    
    translation = np.matrix([[1, 0, 0, x0],#Translation matrix
                             [0, 1, 0, y0],
                             [0, 0, 1, z0],
                             [0, 0, 0,  1]])
    rotation = Rotation_XYZ(roll, pitch, yaw)#rotation matrix
    
    return rotation*translation
    
def Transform(coord,rotation,translation):
    """transforms a vector to a desire rotation and translation"""
    vector = np.array([[coord[0]],
                       [coord[1]],
                       [coord[2]],
                       [      1]])
    
    tranformVector = Tranformationmatrix(rotation,translation)*vector
    return np.array([tranformVector[0,0], tranformVector[1,0], tranformVector[2,0]])