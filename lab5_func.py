
#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm, logm
from lab5_header import *
"""
Angles are in radian, distance are in meters.
"""
def Get_MS():
    # Fill in the correct values for a1~6 and q1~6, as well as the M matrix

    M = np.array(([0, -1, 0, 0.390],
    [0, 0, -1, 0.401],
    [1, 0, 0, 0.2155],
    [0, 0, 0, 1]))
    w1 = np.array([0, 0, 1])
    w2 = np.array([0, 1, 0])
    w3 = np.array([0, 1, 0])
    w4 = np.array([0, 1, 0])
    w5 = np.array([1, 0, 0])
    w6 = np.array([0, 1, 0])
    q1 = np.array([-.150, .150, .010])
    q2 = np.array([-.150, .270, .162])
    q3 = np.array([.094, .270, .162])
    q4 = np.array([.307, .177, .162])
    q5 = np.array([.307, .260, .162])
    q6 = np.array([.390, .260, .162])
    v1 = np.cross(-w1, q1)
    v2 = np.cross(-w2, q2)
    v3 = np.cross(-w3, q3)
    v4 = np.cross(-w4, q4)
    v5 = np.cross(-w5, q5)
    v6 = np.cross(-w6, q6)
    s1 = get_S(w1, v1)
    s2 = get_S(w2, v2)
    s3 = get_S(w3, v3)
    s4 = get_S(w4, v4)
    s5 = get_S(w5, v5)
    s6 = get_S(w6, v6)
    S = np.array([s1,s2,s3,s4,s5,s6])


    return M, S



"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
    # Initialize the return_value
    return_value = [None, None, None, None, None, None]
    print("Foward kinematics calculated:\n")
    theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
    T = np.eye(4)

    M,S =Get_MS()
    T = expm(S[0]*theta1)
    T = np.matmul(T, expm(S[1]*theta2))
    T = np.matmul(T, expm(S[2]*theta3))
    T = np.matmul(T, expm(S[3]*theta4))
    T = np.matmul(T, expm(S[4]*theta5))
    T = np.matmul(T, expm(S[5]*theta6))
    T = np.matmul(T, M)

    print(str(T) + "\n")

    return_value[0] = theta1 + PI
    return_value[1] = theta2
    return_value[2] = theta3
    return_value[3] = theta4 - (0.5*PI)
    return_value[4] = theta5
    return_value[5] = theta6

    return return_value



"""
Function that calculates an elbow up Inverse Kinematic solution for the
UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
    L1 = 0.152
    L2 = 0.120
    L3 = 0.244
    L4 = 0.093
    L5 = 0.213
    L6 = 0.083
    L7 = 0.083
    L8 = 0.082
    L9 = 0.0535
    L10 = 0.059
    xcen = xWgrip-L9*np.cos(PI/180*yaw_WgripDegree)+0.150
    ycen = yWgrip-L9*np.sin(PI/180*yaw_WgripDegree)-0.150
    zcen = zWgrip-0.010-L1

    #print("centers")
    #print(xcen, ycen)
    offset = L2-(L4-L6)
    ang1to6 = np.arctan2(ycen, xcen)
    dist1to6 = np.sqrt(np.square(xcen)+np.square(ycen))
    theta1off = np.arcsin(offset/dist1to6)
    theta1 = ang1to6-theta1off

    #print("intermediate numbers")
    #print(180/PI*ang1to6, 180/PI*theta1off, dist1to6, offset)
    centohelp = np.array([-L7, -(L6+0.027), L10+L8, 1])

    M = np.array(([np.cos(theta1), -np.sin(theta1), 0, xcen],
    [np.sin(theta1), np.cos(theta1), 0, ycen],
    [0, 0, 1, zcen],
    [0, 0, 0, 1]))

    end = np.matmul(M, centohelp)
    dist1toend = np.sqrt(np.square(end[0])+np.square(end[1])+np.square(end[2]))
    ang1toendz = np.arcsin(end[2]/dist1toend)

    theta2help = lawofcos(L3, dist1toend, L5)
    theta3help = lawofcos(L3, L5, dist1toend)

    theta2 = -(ang1toendz + theta2help)
    theta3 = PI-theta3help
    theta4 = -(theta3+theta2)
    theta5 = -PI/2
    theta6 = PI/2-PI/180*yaw_WgripDegree+theta1

    #print("final angles")
    #print(180/PI*theta1, 180/PI*theta2, 180/PI*theta3, 180/PI*theta4, 180/PI*theta5, 180/PI*theta6)

    return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)


def lawofcos(a, b, c):
    return np.arccos((np.square(a)+np.square(b)-np.square(c))/(2*a*b))

def get_S(w, v):
    S = np.array((  [0,     -w[2],  w[1],   v[0]],
                    [w[2],  0,      -w[0],  v[1]],
                    [-w[1], w[0],   0,      v[2]],
                    [0,     0,      0,      0   ]))
    return S

