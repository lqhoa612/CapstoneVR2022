#!/usr/bin/env python
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import rospy
from ur3_unity_ros.srv import *

def plan(req):
    #Initial robot model
    ur3 = rtb.models.UR3()
    #Initial joint angles coordinate
    ur3.q = np.radians([-90, -90, -90, -90, 90, 0])
    # ur3.q = [m.radians(req.unity), m.radians(req.unity), m.radians(req.unity), m.radians(req.unity), m.radians(req.unity), m.radians(req.unity)]
    #Target pose
    Tep = sm.SE3.Trans(req.x, req.z, req.y) * sm.SE3.Eul(np.radians([0, 90, 0]))
    #Get IK solution
    sol = ur3.ikine_LMS(Tep, ur3.q)
    #Convert to degree
    temp = np.degrees(sol[0])
    print(temp)
    #Send response to unity
    res = TrajectoryPlannerResponse()
    res.q = temp
    return res

def Ur3Service():
    rospy.init_node('traj_server')
    s = rospy.Service('traj_planner', TrajectoryPlanner, plan)
    print("Ready to go")

    # #Initial robot model
    # ur3 = rtb.models.UR3()
    # #Initial joint angles coordinate
    # ur3.q = np.radians([-90, -90, -90, -90, 90, 0])
    # # ur3.q = [m.radians(req.unity), m.radians(req.unity), m.radians(req.unity), m.radians(req.unity), m.radians(req.unity), m.radians(req.unity)]
    # #Target pose
    # Tep = sm.SE3.Trans(0.25, 0, 0.1) * sm.SE3.Eul(np.radians([0, 90, 0]))
    # #Get IK solution
    # sol = ur3.ikine_LMS(Tep, ur3.q)
    # #Get trajectory
    # traj = rtb.jtraj(ur3.q, sol[0], 5)
    # # traj.q = np.degrees(traj.q)
    # print(traj.q)
    # #Send response to unity
    # res = TrajectoryPlannerResponse()
    # for i, q in enumerate(traj.q):
    #     temp = JointTrajectoryPoint()
    #     temp.positions = q
    #     res.trajectory.append(temp)

    rospy.spin()

if __name__ == "__main__":
    Ur3Service()