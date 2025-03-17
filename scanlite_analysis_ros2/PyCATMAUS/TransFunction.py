import numpy as np

calibrationm = np.array([[ 0.2,   -0.98, 0, -147.19], 
                         [-0.98,  -0.2,  0,  4.23  ],
                         [ 0,      0,    1, -51    ], 
                         [ 0,      0,    0,  1     ]])
scale = 0.0672

imgp = np.array([[-133.5, -133.5, -171.5, -171.5, -133.5],
                 [  -9.5,  -47.5,  -47.5,   -9.5,   -9.5],
                 [   -51,    -51,    -51,    -51,    -51]])
def transfrom_i2l(img_pts):
    scaled = scale * img_pts
    n_pts = np.shape(scaled)
    scaled = np.vstack([scaled,np.zeros(n_pts[1]),np.ones(n_pts[1])])
    transformed = np.matmul(calibrationm, np.array(scaled))
    return transformed

def quat2rotm (Q):
    qw = Q[0]
    qx = Q[1]
    qy = Q[2]
    qz = Q[3]
    # First row of the rotation matrix
    r00 = 2 * (qw * qw + qx * qx) - 1
    r01 = 2 * (qx * qy - qw * qz)
    r02 = 2 * (qx * qz + qw * qy)
    # Second row of the rotation matrix
    r10 = 2 * (qx * qy + qw * qz)
    r11 = 2 * (qw * qw + qy * qy) - 1
    r12 = 2 * (qy * qz - qw * qx)
    # Third row of the rotation matrix
    r20 = 2 * (qx * qz - qw * qy)
    r21 = 2 * (qy * qz + qw * qx)
    r22 = 2 * (qw * qw + qz * qz) - 1
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])                   
    return rot_matrix