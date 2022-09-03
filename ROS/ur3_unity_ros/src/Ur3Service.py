#!/usr/bin/env python
import roboticstoolbox as rtb
import spatialmath as sm
import math as m
import numpy as np
import rospy
from ur3_unity_ros.srv import *

def plan(req):
    #Initial robot model
    ur3 = rtb.models.UR3()
    #Initial joint angles coordinate
    ur3.q = [m.radians(-90), m.radians(-90), m.radians(-90), m.radians(-90), m.radians(90), m.radians(0)]
    # ur3.q = [m.radians(req.unity), m.radians(req.unity), m.radians(req.unity), m.radians(req.unity), m.radians(req.unity), m.radians(req.unity)]
    #Target pose
    Tep = sm.SE3.Trans(req.x, req.z, req.y) * sm.SE3.Eul([0, m.radians(90), m.radians(0)])
    #Get IK solution
    sol = ur3.ikine_LMS(Tep, ur3.q)
    #Convert rad to deg and round to int
    rs = [  np.round(m.degrees(sol[0][0])), np.round(m.degrees(sol[0][1])), 
            np.round(m.degrees(sol[0][2])), np.round(m.degrees(sol[0][3])),     
            np.round(m.degrees(sol[0][4])), np.round(m.degrees(sol[0][5]))  ]
    
    rs = [  m.degrees(sol[0][0]), m.degrees(sol[0][1]), 
            m.degrees(sol[0][2]), m.degrees(sol[0][3]),     
            m.degrees(sol[0][4]), m.degrees(sol[0][5])  ]
    # ok = rtb.jtraj(ur3.q, sol[0], 4)
    print(rs)
    #Send response to unity
    return TrajPlannerResponse(rs)

def Ur3Service():
    rospy.init_node('traj_server')
    s = rospy.Service('traj_planner', TrajPlanner, plan)
    print("Ready to go")

    # ur3 = rtb.models.UR3()
    # ur3.q = [m.radians(-90), m.radians(-90), m.radians(-90), m.radians(-90), m.radians(90), m.radians(0)]
    # Tep = sm.SE3.Trans(-0.25, 0.25, 0.35) * sm.SE3.Eul([0, m.radians(90), m.radians(0)])
    # sol = ur3.ikine_LMS(Tep, ur3.q)
    # rs = [  m.degrees(sol[0][0]), m.degrees(sol[0][1]), 
    #         m.degrees(sol[0][2]), m.degrees(sol[0][3]),     
    #         m.degrees(sol[0][4]), m.degrees(sol[0][5])]
    # ur3.plot(sol[0])

    rospy.spin()

if __name__ == "__main__":
    Ur3Service()