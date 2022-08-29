import roboticstoolbox as rtb
import swift
import numpy as np
import spatialmath as sm
import spatialgeometry as sg
import math as m


#create swift instance
env = swift.Swift()
env.launch(realtime=True)

#init the model
ur3 = rtb.models.UR3()
ur3.q = [0, m.radians(-90), m.radians(90), m.radians(-90), m.radians(-90), 0]
prev_q = [0,0,0,0,0,0] 

# add ur to swift
env.add(ur3)

#Tep = ur3.fkine(ur3.q) * sm.SE3.Tx(.01) * sm.SE3.Ty(.01) * sm.SE3.Tz(.01) #metres
Tep = sm.SE3.Tx(0.35) * sm.SE3.Ty(0.2) * sm.SE3.Tz(0.15) #metres

axes = sg.Axes(length=0.1, pose=Tep)
env.add(axes)

q = ur3.ikine_LM(Tep, ur3.q)

print(q[0])

arrived = False
dt = 0.01

env.remove(ur3)
ur3.q = q[0]
env.add(ur3)

# while not arrived:
#     # v is a 6 vector representing the spatial error
#     v, arrived = rtb.p_servo(ur3.fkine(ur3.q), Tep, gain=10, threshold=0.01)
#     J = ur3.jacobe(ur3.q)
#     ur3.qd = np.linalg.pinv(J) @ v

#     env.step(dt)

#q = ur3.q
# newq = [m.degrees(q[0]), m.degrees(q[1]), m.degrees(q[2]), m.degrees(q[3]), 
#         m.degrees(q[4]), m.degrees(q[5])]
# print(newq)

# stop the browser tab from closing
env.hold()