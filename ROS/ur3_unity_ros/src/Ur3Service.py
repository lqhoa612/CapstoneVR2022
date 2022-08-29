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
    new_q = ur3.ikine_min(Tep)
    print(new_q)

    return TrajPlannerResponse(new_q)

def Ur3Service():
    rospy.init_node('traj_server')
    s = rospy.Service('traj_planner', TrajPlanner, plan)
    print("Ready to go")
    rospy.spin()

if __name__ == "__main__":
    Ur3Service()