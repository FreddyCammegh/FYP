from roboticstoolbox import Link,ERobot,Robot, Gripper, ELink
import roboticstoolbox as rtb
import numpy as np
'''
# AGV height
h = 150
# elementart transforms of arm, sepparated into joints
E0 = rtb.ET.tz(h)
E0_5 =  rtb.ET.Rz()

E1 = rtb.ET.tz(173.9) 
E2 = rtb.ET.Rz()   

E3 =  rtb.ET.tz(140)
E4 = rtb.ET.Ry()

E5 = rtb.ET.tz(100)
E6 =  rtb.ET.Ry()

E7 = rtb.ET.ty(88.78)
E8 = rtb.ET.Ry()

E9 = rtb.ET.tz(95)
E10 = rtb.ET.Rz()

E11 = rtb.ET.tx(65.5)
E12 = rtb.ET.Rx()
'''
pi = np.pi

# elementart transforms of arm, sepparated into joints
E0 = rtb.ET.tz(1.5)
E0_5 = rtb.ET.tx()


E1 = rtb.ET.tz(1.739) 
E2 = rtb.ET.Rz()   

E3 =  rtb.ET.tz(1.40)
E4 = rtb.ET.Ry()

E5 = rtb.ET.tz(1.00)
E6 =  rtb.ET.Ry()

E7 = rtb.ET.ty(0.8878)
E8 = rtb.ET.Ry()

E9 = rtb.ET.tz(0.95)
E10 = rtb.ET.Rz()

E11 = rtb.ET.tx(-0.655)
E12 = rtb.ET.Rx()
mc =  E0 * E1  * E2 * E3 * E4 * E5 * E6 * E7 * E8 * E9 * E10 * E11 * E12 
# View the ETS

'''
ets1 = rtb.ETS([E1, E2])
ets2 = rtb.ETS([E3, E4])
ets3 = rtb.ETS([E5, E6])
ets4 = rtb.ETS([E7, E8])
ets5 = rtb.ETS([E9, E10])
ets6 = rtb.ETS([E11, E12])
'''
ets0 = rtb.ETS([E0, E0_5])
ets1 = rtb.ETS([E1, E2])
ets2 = rtb.ETS([E4])
ets3 = rtb.ETS([E3, E6])
ets4 = rtb.ETS([E7,E5, E8])

ets5 = rtb.ETS([E9, E10])
ets6 = rtb.ETS([E11, E12])

lp1 = ets1
lp2 = ets1*ets2
lp3 = ets1*ets2*ets3
lp4 = ets1*ets2*ets3*ets4
lp5 = ets1*ets2*ets3*ets4*ets5
lp6 = ets1*ets2*ets3*ets4*ets5*ets6

#Link classes
#Robot class 
L0 = Link(ets0)
L1 = Link(ets1)
L2 = Link(ets2,)
L3 = Link(ets3)
L4 = Link(ets4)
L5 = Link(ets5)
L6 = Link(ets6)

#ERobot class for Mycobot320 with agv as 7th joint
class Mycobot(ERobot):

    def __init__(self):
        super().__init__(
                [L0,L1, L2, L3, L4, L5, L6], name="Mycobot")
        # saved joint joint poses
        self.addconfiguration_attr("qz", np.array([0,0,0,0,0,0,0]))
        self.qr = np.array([0,-0.01064651, -0.38030724,  2.2226768,  -0.3572689,  0,  0])
        self.qf = np.array([0,0.224,0.371,-0.055,-0.89,-0.086,-0.046])
        self.qf2 = np.array([0,-pi*0.8, -0.38030724,  2.2226768,  -0.3572689,  -0.0137881,  -0.01675516])
        self.qz = np.array([2,0,0.05,0,0,0,0])
        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)
        self.addconfiguration("qf", self.qf)
        self.addconfiguration("qf", self.qf2)

    def getJointPos(self):
       # print(self.q)
        self.linkPos = [lp1.fkine(self.q[:1]),lp2.fkine(self.q[:2]),
                        lp3.fkine(self.q[:3]),lp4.fkine(self.q[:4]),lp5.fkine(self.q[:5])
                        ,lp6.fkine(self.q[:6])]
        
        
        return self.linkPos
    


q = np.array([0,0,0,0,0,0,0])


r=Mycobot()

r.plot(r.qz,loop=True)