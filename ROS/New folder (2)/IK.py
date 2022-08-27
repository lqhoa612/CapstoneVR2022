import numpy as np

X = 5
Y = 0

theta2 = np.arctan2(Y,X)

R0_6 = [[-1,0,0],
        [0,-1,0],
        [0,0,-1]]

R0_3 = [[-np.sin(theta2), 0, np.cos(theta2)],
        [np.cos(theta2), 0, np.sin(theta2)],
        [0,1,0]]

invR0_3 = np.linalg.inv(R0_3)

R3_6 = np.dot(invR0_3,R0_6)

print('R3_6 = ',np.matrix(R3_6))

theta5 = np.arccos(R3_6[2][2])
theta6 = np.arccos(-R3_6[2][0]/np.sin(theta5))
theta4 = np. arccos(R3_6[1][2]/np.sin(theta5))

R3_6_check =   [[-np.sin(theta4)*np.cos(theta5)*np.cos(theta6)-np.cos(theta4)*np.sin(theta6), np.sin(theta4)    *np. cos(theta5)*np.sin(theta6)-np.cos(theta4)*np.cos(theta6), -np.sin(theta4)*np.sin(theta5)],

                [np.cos(theta4)*np.cos(theta5)*np.cos(theta6)-np.sin(theta4)*np.sin(theta6), -np.cos(theta4)*np.cos(theta5)*np.sin(theta6)-np.sin(theta4)*np.cos(theta6), np.cos(theta4)*np.sin(theta5)],

                [-np.sin(theta5)*np.cos(theta6), np.sin(theta5)*np.sin(theta6)+np.cos(theta5), np.cos(theta5)]]

print(np.matrix(R3_6_check))