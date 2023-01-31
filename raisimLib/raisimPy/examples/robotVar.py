import numpy as np

L0=0.136
L1=0.149
L2=0.45
L3=0.249

# alpha a d theta
def DH_parameter(joints):
    q1, q2, q3 = np.array(joints).T
    return np.array([[0,   0,   L1+L0,     q1],
                     [-np.pi/2, 0,   0,      q2-90*np.pi/180],
                     [0 ,       L2,  0,      q3],
                     [0 ,       L3,   0,      0]
                     ])
