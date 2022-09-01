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
ur3.q = [m.radians(-90), m.radians(-90), m.radians(-90), m.radians(-90), m.radians(90), m.radians(0)]
# prev_q = [m.radians(-90), m.radians(-90), 0, 0, 0, 0]

# add ur to swift
env.add(ur3)
Tep = sm.SE3.Trans(-.3, -.1, 0.3) * sm.SE3.Eul([m.radians(0), m.radians(90), m.radians(0)])
axes = sg.Axes(length=0.1, pose=Tep)
env.add(axes)

q = ur3.ikine_LMS(Tep, ur3.q)
ok = rtb.jtraj(ur3.q, q[0], 5)
print(q)

env.remove(ur3)
ur3.q = q[0]
env.add(ur3)
rs = [  m.degrees(q[0][0]), m.degrees(q[0][1]), 
            m.degrees(q[0][2]), m.degrees(q[0][3]),     
            m.degrees(q[0][4]), m.degrees(q[0][5])]
print(rs)

env.hold()