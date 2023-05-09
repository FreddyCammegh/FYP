import asyncio
from cmath import atan
import websockets
import math
# websockets.unicode = str
import asyncio
import numpy as np
import time
import itertools
from pymycobot.genre import Coord
#from pymycobot.mycobot import MyCobot
from Mycobot320dof6V2 import Mycobot
import roboticstoolbox as rtb
import spatialmath as sm
import spatialgeometry as sg
from spatialmath import SE3
from controlFunctions import performTrasform, rotationTransform, angle_axis
#mc = MyCobot('/dev/ttyAMA0')
ready_pos = [-0.61, -21.79, 127.35, -20.47, -0.79, -0.96]
RANGE=200
RANGE_LOWER=120
TCameraGripper = np.array([[0],[0],[0],[0]])
robot = Mycobot()
gain = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
# Initialise finite state variables
CAN_SEE_STRAWBERRY = False
STRAWBERRY_IN_RANGE = False
CAN_SEE_STRAWBERRY_IN_RANGE =False
REACHED_TARGET = False
MSG_IS_CV_CLIENT = bool
MSG_IS_MYCOBOT = bool
TARGET_NEEDS_UPDATE = True
ROBOT_IS_GRASPING = False
ROBOT_IS_WAITING = False
GO_TO_QREADY = True 
GRIPPER_ACTIVE = False
DELIVER_FRUIT = False
GRIPPER_CLOSE = False
GRIPPER_OPEN = False

strawberry_c = list()
joint_angles = list()
joint_velocity = int()
last_seen_target = list()
validTarget = [0,0,0]    
# Only the translation axes
trans_axes = [True, True, True, False, False, False]
# Only the rotation aces
rot_axes = [False, False, False, True, True, True]
# All axes
all_axes = [True, True, True, True, True, True]
# in mm and radians
E1 = rtb.ET.tz(173.9)
E2 = rtb.ET.Rz()        #
E3 = rtb.ET.Ry()        

E4 = rtb.ET.tz(140)
E5 = rtb.ET.Ry()

E6 = rtb.ET.tz(100)
E7 = rtb.ET.Ry()

E8 = rtb.ET.ty(88.78)

E9 = rtb.ET.tz(95)
E10 = rtb.ET.Rz()

E11 = rtb.ET.tx(65.5)
E12 = rtb.ET.Rx()
last_seen_target = [0,0,0]

#T#c = sm.SE3.Trans(-0.443, 0, 0.1) * sm.SE3.RPY(1,1,1)
#print(Tc)

Tg = sm.SE3.Trans(-0.4, 0,1) #* sm.SE3.RPY(1,1,1) #* sm.SE3.Trans(0, 0, 1)
#robotold = E1 * E2 * E3 * E4 * E5 * E6 * E7 * E8 * E9 * E10 * E11 * E12 
print(robot)
locations = [[0,1,1]]
target = 0
pi = np.pi
def rotationMatrixToEulerAngles(R) :
 
 
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
 
    singular = sy < 1e-6
 
    if  not singular: 
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

def degToRad(angles):
    rad = []
    for i in angles:
        rad.append(math.radians(i))
    return np.array(rad)
def convertToDec(targ):
    new_targ = []
    for i in targ:
        new_targ.append(i/100)
    return new_targ
def sendListMessage(list):
    data=str(list[0])+' '+str(list[1])+' '+str(list[2])+' '+str(list[3])+' '+str(list[4])+' '+str(list[5])
    return data
# Define a callback function to handle incoming messages from clients
def PositionBasedServoing(joint_angles,target,ee_axes):
    Tep = target
    J = robot.jacob0(joint_angles)
    Te = robot.fkine(joint_angles).A
    #print('Te; ',Te)
    J_inv = np.linalg.pinv(J)
    #print('teps: ',Te,Tep)
    #e1 = angle_axis(Te, Tep)
    #print('error:',e1)
    ev, arrived = rtb.p_servo(Te, Tep, gain=gain, threshold=0.1,method='angle-axis')
    qDOT = J_inv @ ev
    ee_axes.T = Te
    control_msg = str(sendListMessage(qDOT))
    #print('arrived: ', arrived)
    return control_msg, arrived


async def message_handler(websocket, path):
    #print('CONNECTED...')
    global CAN_SEE_STRAWBERRY, STRAWBERRY_IN_RANGE ,REACHED_TARGET ,MSG_IS_CV_CLIENT ,MSG_IS_MYCOBOT ,TARGET_NEEDS_UPDATE ,ROBOT_IS_GRASPING,GO_TO_QREADY, CAN_SEE_STRAWBERRY_IN_RANGE,ROBOT_IS_WAITING,GRIPPER_ACTIVE,DELIVER_FRUIT,GRIPPER_CLOSE,GRIPPER_OPEN,RANGE_LOWER
    global joint_angles,last_seen_target,control_msg,Tep,validTarget
    # Wait for a message from the client
    
    message=await websocket.recv()
    message_split=message.split()
    message_split = list(map(float, message_split))
    #print('message: ',message)
    '''
ROBOT_IS_GRASPING
HAS ARRIVED AT FRUIT!
#####-GRIPPING-#####
#####-DELIVERING-#####
CONTROL MESSAGE:
'''
    if len(message_split)==7:
        MSG_IS_CV_CLIENT = False
        MSG_IS_MYCOBOT = True
        CAN_SEE_STRAWBERRY = CAN_SEE_STRAWBERRY
        # cvClient.py
        joint_angles = message_split[:-1]
        #print(joint_angles)
        #robot.d = np.array([0,0,0,0,0,0])
        joint_angles=degToRad(joint_angles)
        #print(joint_angles)
    
        joint_velocity = message_split[-1]
    if len(message_split)==3:
        # [DEPTH, X, Y]
        message_split[0] = abs(message_split[0])
        MSG_IS_CV_CLIENT = True
        MSG_IS_MYCOBOT = False
        print('message: ',message_split)
        if message_split[0]==0:
            CAN_SEE_STRAWBERRY = False
            STRAWBERRY_IN_RANGE = False
        elif message_split[0]!=0:
            #CAN_SEE_STRAWBERRY = True
            if message_split[0]< RANGE and message_split[0] > RANGE_LOWER:
                STRAWBERRY_IN_RANGE = True
                CAN_SEE_STRAWBERRY_IN_RANGE = True
                
                ##print(message_split)
                last_seen_target=message_split
                locations.append(last_seen_target)
                validTarget = convertToDec(message_split)
                #validTarget = [-validTarget[1],0,0] #-validTarget[1]-0.3,validTarget[2]+0.25,validTarget[0]-0.65
            elif message_split[0]> RANGE:
                STRAWBERRY_IN_RANGE = False

        else:
            print('cvClient.py sent erroneous message...')
      
      
    # Defining relevant reference frames
    ee_axes = sg.Axes(0.1)
    target = sg.Axes(0.1)
    ee_axes.T=robot.fkine(joint_angles) 
    


    if GRIPPER_CLOSE == True:
        print('#####-GRIPPING-#####')
        #await websocket.send('888')
        control_msg = '888'
        #time.sleep(5)
        GRIPPER_CLOSE = False
        DELIVER_FRUIT = True

    #print('in the loop')
    if GO_TO_QREADY == True:
        print('GO_TO_QREADY')
        Tep= robot.fkine(robot.qr)
        control_msg, arrived = PositionBasedServoing(joint_angles, Tep, ee_axes)
        if arrived :
            GO_TO_QREADY = False
            ROBOT_IS_WAITING = True
        #print('MSG to control:', control_msg)q
        #await websocket.send(control_msg)
    

    if ROBOT_IS_WAITING == True: 
        print('#####-ROBOT_IS_WAITING-#####')
        if CAN_SEE_STRAWBERRY_IN_RANGE == True:
            ROBOT_IS_GRASPING = True
            ROBOT_IS_WAITING = False
        else:
            control_msg = '999'
            #await websocket.send('999')

    elif ROBOT_IS_GRASPING :
        print('ROBOT_IS_GRASPING')
        #print('TARGET_NEEDS_UPDATE:', TARGET_NEEDS_UPDATE)
        #print(target)
        #print('valid target: ', validTarget)
        if TARGET_NEEDS_UPDATE == True:
            #Tep = robot.fkine(joint_angles)*SE3.Trans([ 0,validTarget[0],validTarget[2]])*SE3.Trans([ 0,0,-0.8]).A
            Tep = robot.fkine(joint_angles)*SE3.Trans([-validTarget[2],-validTarget[1],validTarget[0]])*Tg#*SE3.RPY([0,0,-pi/10])#SE3.Trans(validTarget)      # SE3.RPY(1,1,1).A
            #Tep= robot.fkine(joint_angles)*SE3.Trans(validTarget).A#,float(locations[0][1]),float(locations[0][2])).A
            #print('PICKED TEP!')
            TARGET_NEEDS_UPDATE = False
        euler = rotationMatrixToEulerAngles(Tep.R)
        #Tep = Tep.A
        print('euler: ',euler)
        print('translation: ',Tep.t)
        control_msg, arrived = PositionBasedServoing(joint_angles, Tep.A, ee_axes)
        #await websocket.send(control_msg)
        if arrived :
            print('HAS ARRIVED AT FRUIT!')
            REACHED_TARGET = True
            CAN_SEE_STRAWBERRY_IN_RANGE = False
            TARGET_NEEDS_UPDATE = True
            validTarget = [0,0,0]
            ROBOT_IS_GRASPING = False
            target = 0
           
            GO_TO_QREADY = False
            GRIPPER_CLOSE = True
            control_msg = '888'


    

    if DELIVER_FRUIT == True:
        print('#####-DELIVERING-#####')
        # once complete
        
        Tep = robot.fkine(robot.qr).A
        control_msg, arrived = PositionBasedServoing(joint_angles, Tep, ee_axes)
        #await websocket.send(control_msg)
        if arrived :
            print('HAS ARRIVED AT FRUIT!')
            DELIVER_FRUIT = False
            validTarget = [0,0,0]
            GRIPPER_OPEN = True
            

    if GRIPPER_OPEN == True:
        print('#####-GRIPPING-#####')
        #await websocket.send('888')
        control_msg = '777'
        #time.sleep(5)
        GRIPPER_OPEN = False
        GO_TO_QREADY = True
        time.sleep(2.5)


    #print('CONTROL MESSAGE: ',control_msg)
    await websocket.send(control_msg)
    #cameraFrame=[abs(float(message_split[0])*10),float(message_split[1])*10,float(message_split[2])*10] #[d,x,y]
    # depth,xCoord,yCoord
    #objCameraFrame=np.array([[cameraFrame[0]],[-cameraFrame[1]],[cameraFrame[2]],[1]])
    #eeTranslation, eeRotation, roll, pitch, yaw = getEEInfo()

    #print(f"Received message from client: {message}")

    # Send a response back to the client
    #response = "Hello, from controller!"
    #print(message_split[0])
    #print(last_seen_target)
    
    
    
# Start the server


# Start the server in the asyncio event loop
ip_address = '172.20.10.3'  # Bind to all available network interfaces
port = 8777
server = websockets.serve(message_handler, ip_address, port)
asyncio.get_event_loop().run_until_complete(server)
print('hi')
asyncio.get_event_loop().run_forever()

