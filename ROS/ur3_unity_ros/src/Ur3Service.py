#!/usr/bin/env python
import roboticstoolbox as rtb
import spatialmath as sm
import spatialgeometry as sg
import math as m
import numpy as np
import rospy

from ur3_unity_ros.srv import *

def plan(req):
    #Init the model
    ur3 = rtb.models.UR3()
    #Match the unity pose and save as previous pose
    prev_q = ur3.q = [m.radians(-90), m.radians(-90), 0, 0, 0, 0]
    #Get the desired pose
    Tep = sm.SE3.Trans(req.x, req.y, req.x)
    sol = ur3.ikine_LM(Tep)
    rs = [m.degrees(sol.q[0]), m.degrees(sol.q[1]), m.degrees(sol.q[2]),
            m.degrees(sol.q[3]), m.degrees(sol.q[4]), m.degrees(sol.q[5])]

    new_q = [np.round(rs[0],2), np.round(rs[1],2), np.round(rs[2],2),
            np.round(rs[3],2), np.round(rs[4],2), np.round(rs[5],2)]

    print(new_q)

    return TrajPlannerResponse(new_q)

def Ur3Service():
    rospy.init_node('traj_server')
    s = rospy.Service('traj_planner', TrajPlanner, plan)
    print("Ready to go")
    rospy.spin()

if __name__ == "__main__":
    Ur3Service()