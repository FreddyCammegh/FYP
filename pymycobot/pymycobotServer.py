from cmath import atan
import websockets
import math
# websockets.unicode = str
import asyncio
import numpy as np
import time
import itertools
from pymycobot.genre import Coord
from pymycobot.mycobot import MyCobot

mc = MyCobot('/dev/ttyAMA0')

#ready_pos = [-4.57, -42.45, 88.24, 41.74, -3.16, 4.48]
ready_pos = [-0.61, -21.79, 127.35, -20.47, -0.79, -0.96]
RANGE=200
TCameraGripper = np.array([[0],[0],[0],[0]])

def gripper_test2():
    #flag = mc.is_gripper_moving()
    #print('Is gripper moving:' ,flag)
    mc.set_gripper_value(52,70)
    time.sleep(1)
    mc.set_gripper_value(20,70)
    time.sleep(1)
    #mc.set_gripper_state(0,70)
    time.sleep(1)
  
   # mc.send_angles([152.31, 75.84, 46.66, 33.39, -7.73, -0.87],15)
    mc.send_angles(ready_pos,20)
    time.sleep(2)
    mc.set_gripper_value(52,70)
    time.sleep(1)

def rotationTransformOLD(roll,pitch,yaw):
    R_x = np.array([[1,                0,               0],
                    [0,   math.cos(roll), -math.sin(roll)],
                    [0,   math.sin(roll),  math.cos(roll)]])

    R_y = np.array([[math.cos(pitch),         0,  math.sin(pitch)],
                    [0,                       1,                0],
                    [-math.sin(pitch),       0,  math.cos(pitch)]])

    R_z = np.array([[math.cos(yaw),   -math.sin(yaw),           0],
                    [0,       math.sin(yaw),          math.cos(yaw)],
                    [0,                 0,              1]])

    R_T = np.dot(np.dot(R_z,R_y),R_x)
    return R_T

def rotationTransform(roll,pitch,yaw):
    R_x = np.array([[1,                0,               0],
                    [0,   math.cos(roll), -math.sin(roll)],
                    [0,   math.sin(roll),  math.cos(roll)]])

    R_y = np.array([[math.cos(pitch),         0,  math.sin(pitch)],
                    [0,                       1,                0],
                    [-math.sin(pitch),       0,  math.cos(pitch)]])

    R_z = np.array([[math.cos(yaw),   -math.sin(yaw),           0],
                    [math.sin(yaw),    math.cos(yaw),           0],
                    [0,                            0,           1]])

    R_T = np.dot(np.dot(R_x,R_y),R_z)
    return R_T

def performTrasform(R_T, objCameraFrame, ee_translation):
    #print('R_T', R_T)
    #print('obj', objCameraFrame)
    #print('ee trans', ee_translation)
    transform = np.block([[R_T,ee_translation],[np.zeros((1,3)),1]])
    Pc=objCameraFrame
    Pr=np.dot(transform,Pc)
    return Pr

def getEEInfo():
    coords = mc.get_coords()
    #print('coords: ',coords)
    eeTranslation = np.array([[coords[0]+80],[coords[1]],[coords[2]+85]])
    eeRotation = [coords[3],coords[4],coords[5]]
    roll=math.radians(coords[3])
    pitch=math.radians(coords[4])
    yaw=math.radians(coords[5]) - math.pi/2
    #roll=coords[3]
    #pitch=coords[4]
    #yaw=coords[5]
    return eeTranslation, eeRotation, roll, pitch, yaw


def graspTargetT(eeTranslation, roll, pitch, yaw,objCameraFrame):
    R_T=rotationTransform(roll,pitch,yaw)
    targetR = performTrasform(R_T,objCameraFrame,eeTranslation)
    targetR = np.add(targetR,TCameraGripper)
    targetArray=targetR.tolist()
    target= list(itertools.chain(*targetArray))
    target.pop()
    rTarget=[]
    for i in target:
        rTarget.append(round(i,3))
    #print('Target: ',rTarget)
    return rTarget



def graspTarget(data):
    x_t=round(data[0],3)
    y_t=round(data[1],3)
    z_t=round(data[2],3)
    #print('xyzt',x_t,y_t,z_t)
    c_pos = mc.get_coords()
    time.sleep(0.05)
    if len(c_pos) == 0:
        print('STILL FINDING CURRENT POSITION')
    elif len(c_pos) > 1:
        #print('cpos: ' ,c_pos,'data: ' ,data)
        #target_pos=[round((c_pos[0]-x_t)+95,3),round(c_pos[1]+y_t,3),round((c_pos[2]+z_t)+100,3),c_pos[3],c_pos[4],c_pos[5]]
        target_pos=[round((c_pos[0]-x_t),3),round(c_pos[1]+y_t,3),round((c_pos[2]+z_t),3),c_pos[3],c_pos[4],c_pos[5]]

        #print('c/target',c_pos,target_pos)
        #mc.send_coords(target_pos,10,0)
        #time.sleep(3)
        #gripper_test2()
        return target_pos

def calculateAngles(objectPos): #=[x,y,z]
    z=objectPos[0]
    x=objectPos[1]
    y=objectPos[2]
    print(x,y,z)
    yaw=math.degrees(-math.atan(x/z))
    roll=math.degrees(-math.atan(y/z))




    return roll,yaw


async def hello(websocket, path):
    c_pos=mc.get_coords()
    message=await websocket.recv()
    message_split=message.split()
    cameraFrame=[abs(float(message_split[0])*10),float(message_split[1])*10,float(message_split[2])*10]  #[d,x,y]
    # depth,xCoord,yCoord
    objCameraFrame=np.array([[cameraFrame[0]],[-cameraFrame[1]],[cameraFrame[2]],[1]])

    #objCameraFrame2=np.array([[cameraFrame[2]],[cameraFrame[0]],[cameraFrame[1]],[1]])
    print('Camera frame: ',cameraFrame)
    eeTranslation, eeRotation, roll, pitch, yaw = getEEInfo()
    #roll, pitch, yaw = -4.93, 0.7, -91.29
    if cameraFrame[0]!=0 and abs(cameraFrame[0])<RANGE:
        print('/////////////////Grasping strawberry!////////////////////')
        #graspTarget(cameraFrame)
        #targetTestXYZ = graspTargetT(eeTranslation, roll, pitch, yaw,objCameraFrame2)
        targetXYZ = graspTargetT(eeTranslation, roll, pitch, yaw,objCameraFrame)
        target=targetXYZ + eeRotation
        #targetTest = targetTestXYZ + eeRotation

        #c_pos[0] = target[0]
        #c_pos[2] = target[2]
        #target = c_pos

        print('final target: ', target)
        #print('final targt2: ', targetTest)
        #print('accurate target:',graspTarget(cameraFrame))
        mc.send_coords(target,20,1)
        #time.sleep(3)
        #gripper_test2()
        #print('actual position ',mc.get_coords())
        #print('accurate test target:',mc.get_angles())
        mc.send_angles(ready_pos,20)
        
    elif abs(cameraFrame[0])>=RANGE:
        print('//////////////////Can see strawberry!////////////////////')
        roll, yaw =calculateAngles(cameraFrame)
        #print('roll and yaw:',roll,yaw)
        angles=mc.get_angles()
        #print('Current angles: ',angles)
      
        #angles[5]=angles[5]+yaw
        #print('New angles:',angles)
        #mc.send_angles(angles,20)
        #time.sleep(0.1)
        

    else :
        #print('No strawberry!')
        mc.send_angles(ready_pos,120)
    
# print('7')  
#Use the ip which is under wlan inet
start_server = websockets.serve(hello, host = '172.20.10.4' , port = 7890) #'172.20.10.2'  '172.0.0.1'
# print('8')
asyncio.get_event_loop().run_until_complete(start_server)
# print('9')
asyncio.get_event_loop().run_forever()


while 1:
    print('TEST')