#!/usr/bin/env python
from ast import For
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
    prev_q = req.unity
    # prev_q = [  m.radians(prev_q[0]), m.radians(prev_q[1]),
    #             m.radians(prev_q[2]), m.radians(prev_q[3]),
    #             m.radians(prev_q[4]), m.radians(prev_q[5])  ]
    # prev_q[0] += m.radians(-90)
    # prev_q[1] += m.radians(-90)
    # prev_q[2] += m.radians(-90)
    # prev_q[3] += m.radians(-90)
    # prev_q[4] += m.radians(90)

    ur3.q = [m.radians(-90), m.radians(-90), m.radians(-90), m.radians(-90), m.radians(90), m.radians(0)]
    Tep = sm.SE3.Trans(-0.25, 0.25, 0.2) * sm.SE3.Eul([0, m.radians(90), m.radians(0)])
    sol = ur3.ikine_LMS(Tep, ur3.q)
    rs = [  m.degrees(sol[0][0]), m.degrees(sol[0][1]), 
            m.degrees(sol[0][2]), m.degrees(sol[0][3]),     
            m.degrees(sol[0][4]), m.degrees(sol[0][5])]
    
    # ok = rtb.jtraj(ur3.q, sol[0], 4)
    print(rs)
    #msg = TrajPlannerResponse(rs)
    #msg.ros = rs
    # res = []
    # res.insert(msg)
    # for i in range(len(res)):
    #     res[i].data = ok.q[i]

    return TrajPlannerResponse(rs)

def Ur3Service():
    rospy.init_node('traj_server')
    s = rospy.Service('traj_planner', TrajPlanner, plan)
    print("Ready to go")

    # ur3 = rtb.models.UR3()
    # ur3.q = [m.radians(-90), m.radians(-90), m.radians(-90), m.radians(-90), m.radians(90), m.radians(0)]
    # Tep = sm.SE3.Trans(-0.25, 0.25, 0.2) * sm.SE3.Eul([0, m.radians(90), m.radians(0)])
    # sol = ur3.ikine_LMS(Tep, ur3.q)
    # rs = [  m.degrees(sol[0][0]), m.degrees(sol[0][1]), 
    #         m.degrees(sol[0][2]), m.degrees(sol[0][3]),     
    #         m.degrees(sol[0][4]), m.degrees(sol[0][5])]
    # print(rs)


    rospy.spin()

if __name__ == "__main__":
    Ur3Service()