import websockets
import asyncio
import torch
import cv2
import time
import numpy as np
import math
def find_depth(right_point, left_point, frame_right, frame_left, baseline,frameDims):
    height = frameDims[0]
    width = frameDims[1]
    PPL = 0.05   #0.2
    # CONVERT FOCAL LENGTH f FROM [mm] TO [pixel]:
    width_right = frame_right.shape[1]
    width_left= frame_left.shape[1]

    x_right = right_point[0]
    x_left = left_point[0]

    # CALCULATE THE DISPARITY:
    disparity = x_left-x_right      #Displacement between left and right frames [pixels]
    f_pixel = 80/PPL
    # CALCULATE DEPTH z:
    #zDepth = (baseline*f_pixel)/(disparity*0.3333)  
    zDepth = (baseline*f_pixel)/(disparity)            #Depth in [cm]
          #Depth in [cm]

    return zDepth

# calibration information
calibrationResolution = (480,640)
mtx_left = np.array([[625.1233004546057, 0.0, 383.108183183195 ],
                [0.0, 627.5696121658324, 240.69218605957343 ],
                [0.0, 0.0, 1.0 ]])
mtx_right = np.array([[571.885410325627, 0.0, 358.6458641032337 ],
                [0.0, 574.2779692236742, 262.3215106381796 ],
                [0.0, 0.0, 1.0 ]])
R = np.array([
[0.9988266345779173, 0.0020223232143798987, 0.04838661247254422 ],
[-0.0007931259454983818, 0.9996768146994754 ,-0.025409390066376707 ],
[-0.04842236063004539, 0.025341198888909858, 0.9985054324488608 ]])
T = np.array([-5.978408247849559 ,0.20182342062623754 ,-3.594597560863502 ])

E = np.array([
[0.9988266345779173, 0.0020223232143798987, 0.04838661247254422 ,-5.978408247849559],
[-0.0007931259454983818, 0.9996768146994754 ,-0.025409390066376707,0.20182342062623754 ],
[-0.04842236063004539, 0.025341198888909858, 0.9985054324488608,-3.594597560863502 ]])
baseline = 6
# set frame resolution
height = 720
width = 2560
# Open both cameras
cap= cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)

# Rescale camera amatrix
print(mtx_left)
mxScalar = calibrationResolution[0]/height
myScalar = calibrationResolution[1]/width
mtx_left[0] =mtx_left[0] / mxScalar
mtx_left[1] =mtx_left[1] / myScalar
#Colour masks
lower_red = np.array([0, 50, 50])
upper_red = np.array([10, 255, 255])
lower_red2 = np.array([170, 50, 50])
upper_red2 = np.array([180, 255, 255])

lower_green = np.array([40, 50, 50])
upper_green = np.array([80, 255, 255])
# set frame resolution
height = 720
width = 2560
if torch.cuda.is_available():  
  print('Using GPU')
  dev = "cuda:0" 
else:  
  dev = "cpu" 
  print('Using CPU')
async def send_message():
    reader, writer = await asyncio.open_connection('localhost', 8888)
    message = 'Hello, world!'
    writer.write(message.encode())
    await writer.drain()
    writer.close()
    await writer.wait_closed()
async def dataSender():
    model = torch.hub.load('ultralytics/yolov5', 'custom', path='weights/best.pt', force_reload=True) 
    
    while(cap.isOpened()):
        url = "172.20.10.3:7890" 
        ip_address = '172.20.10.3'  # Replace with the IP address of the server
        port = 8777   #use mycobot ip address
        async with websockets.connect(f"ws://{ip_address}:{port}") as ws:
        
        #print('here!!!!')
        #url = "ws://172.20.10.4:7890"    #use mycobot ip address
        #async with websockets.connect(url,ping_interval=None) as ws:
            
            #send_message()
            s,original = cap.read()
            #original = cv2.resize(original,(640,240))
            frame_left=original[0:height,0:int(width/2)]
            frame_right=original[0:height,int(width/2):(width)]
            #print(np.shape(original), np.shape(frame_left),np.shape(frame_right))
            #start1=time.time()
              # Convert the BGR image to RGB
            frame_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2RGB)
            frame_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2RGB)
            #print('dims: ', frame_right.shape[0],frame_right.shape[1])

            # Process the image and find fruit
            results_right = model(frame_right)
            results_left = model(frame_left)
            data = []
            # Convert the RGB image to BGR
            frame_right = cv2.cvtColor(frame_right, cv2.COLOR_RGB2BGR)
            frame_left = cv2.cvtColor(frame_left, cv2.COLOR_RGB2BGR)
            if results_left and results_right: #.Detections:
              center_point_left=0
              x=0
              y=0
              for detectionL, detectionR in zip(results_left.xyxyn[0],results_right.xyxyn[0]):
                  labels= detectionL[-1]
                  boxCordL = detectionL[:-2]
                  boxCordR = detectionR[:-2]
                  confidence=detectionL[-2]
                  if confidence > 0.7:
                      #print(frame_left.shape)
                      h, w, c = frame_left.shape
                      boundBoxL = int(boxCordL[0] * w), int(boxCordL[1] * h), int(boxCordL[2] * w), int(boxCordL[3] * h)
                      center_point_left = (boundBoxL[0] + boundBoxL[2] / 2, boundBoxL[1] + boundBoxL[3] / 2)
                      boundBoxR = int(boxCordR[0] * w), int(boxCordR[1] * h), int(boxCordR[2] * w), int(boxCordR[3] * h)
                      center_point_right = (boundBoxR[0] + boundBoxR[2] / 2, boundBoxR[1] + boundBoxR[3] / 2)
                      # calculate the depth of the feature
                      depth = find_depth(center_point_right, center_point_left, frame_right, frame_left, baseline,(height,width))
                      depth=round(depth,2)
                      u,v= center_point_left[0],center_point_left[1]

                      maskL = np.zeros(frame_left.shape[:2], np.uint8)
                      cv2.rectangle(maskL, (int(boundBoxL[0]),int(boundBoxL[1])), (int(boundBoxL[2]),int(boundBoxL[3])), 255, -1)
                      maskR = np.zeros(frame_left.shape[:2], np.uint8)
                      cv2.rectangle(maskR, (int(boundBoxR[0]),int(boundBoxR[1])), (int(boundBoxR[2]),int(boundBoxR[3])), 255, -1)

                      #print(mtx_left)
                      masked_bb_imgL = cv2.bitwise_and(frame_left,frame_left, mask=maskL)
                      masked_bb_imgR = cv2.bitwise_and(frame_right,frame_right, mask=maskR)
                      masked_bb_imgL_HSV = cv2.cvtColor( masked_bb_imgL , cv2.COLOR_BGR2HSV)
                      masked_bb_imgR_HSV = cv2.cvtColor( masked_bb_imgR , cv2.COLOR_BGR2HSV)
                      masked_bb_imgL_HSV_grey = cv2.cvtColor( masked_bb_imgL , cv2.COLOR_BGR2GRAY)
                      masked_bb_imgR_HSV_grey = cv2.cvtColor( masked_bb_imgR , cv2.COLOR_BGR2GRAY)
                      masked_imgL_HSV = cv2.cvtColor( frame_left , cv2.COLOR_BGR2HSV)
                      masked_imgR_HSV = cv2.cvtColor( frame_right , cv2.COLOR_BGR2HSV)
                      mask1L = cv2.inRange(masked_bb_imgL_HSV, lower_red, upper_red)
                      mask2L = cv2.inRange(masked_bb_imgL_HSV, lower_red2, upper_red2)
                      mask1R = cv2.inRange(masked_bb_imgR_HSV, lower_red, upper_red)
                      mask2R = cv2.inRange(masked_bb_imgR_HSV, lower_red2, upper_red2)
                      CmaskL = cv2.bitwise_or(mask1L, mask2L)
                      CmaskR = cv2.bitwise_or(mask1R, mask2R)
                      red_pixelsL = cv2.bitwise_and(masked_bb_imgL_HSV,masked_bb_imgL_HSV, CmaskL)
                      red_pixelsR = cv2.bitwise_and(masked_bb_imgR_HSV,masked_bb_imgR_HSV, CmaskR)
                      dual_redmask =cv2.bitwise_or(red_pixelsL,red_pixelsR)
                      REDcom = [ np.average(indices) for indices in np.where(red_pixelsL >= 255) ]
                      #red_pixelsg = cv2.cvtColor(red_pixels, cv2.COLOR_BGR2GRAY)
                      # Green L/R
                      green_pixelsL = cv2.inRange(masked_bb_imgL, lower_green, upper_green)
                      green_pixelsR = cv2.inRange(masked_bb_imgL, lower_green, upper_green)
                      dual_greenmask =cv2.bitwise_or(green_pixelsL,green_pixelsR)
                      GREENcom = [ np.average(indices) for indices in np.where(green_pixelsL >= 255) ]
                      #colour bands
                      redLower1 = np.array([0, 70, 50])
                      redUpper1 = np.array([10, 255, 255])
                      redLower2 = np.array([170, 20, 50])
                      redUpper2 = np.array([180, 255, 255])
                      greenLower = np.array([36, 25, 25])
                      greenUpper = np.array([70, 255, 255])
                      #Colour masks
                      RedmaskL1 = cv2.inRange(masked_bb_imgL_HSV, redLower1,redUpper1)
                      RedmaskR1 = cv2.inRange(masked_bb_imgR_HSV, redLower1,redUpper1)
                      RedmaskL2 = cv2.inRange(masked_bb_imgL_HSV, redLower2,redUpper2)
                      RedmaskR2 = cv2.inRange(masked_bb_imgR_HSV, redLower2,redUpper2)
                      GreenmaskL = cv2.inRange(masked_bb_imgL_HSV, greenLower,greenUpper)
                      GreenmaskR = cv2.inRange(masked_bb_imgL_HSV,greenLower,greenUpper)

                      RedmaskL = cv2.bitwise_or(RedmaskL1, RedmaskL2)
                      #cv2.imshow('test redmask', RedmaskL)
                      RedmaskR = cv2.bitwise_or(RedmaskR1, RedmaskR2)
                      #Applying mask to frame
                      red_pixelsL = cv2.bitwise_and(masked_bb_imgL_HSV,masked_bb_imgL_HSV, RedmaskL)
                      red_pixelsR = cv2.bitwise_and(masked_bb_imgR_HSV,masked_bb_imgR_HSV, RedmaskR)
                      grey_red_pixelsL = cv2.bitwise_and(masked_bb_imgL_HSV_grey,masked_bb_imgL_HSV_grey,RedmaskL)#convertToGrey(red_pixelsL)
                      grey_red_pixelsR = cv2.bitwise_and(masked_bb_imgR_HSV_grey,masked_bb_imgR_HSV_grey,RedmaskR)##convertToGrey(red_pixelsR)
                      cv2.imshow('greytestL', grey_red_pixelsL)
                      cv2.imshow('greytestR', grey_red_pixelsR)

                      green_pixelsL = cv2.bitwise_and(masked_bb_imgL_HSV,masked_bb_imgL_HSV, GreenmaskL)
                      green_pixelsR = cv2.bitwise_and(masked_bb_imgR_HSV,masked_bb_imgR_HSV, GreenmaskR)

                      

                      #redMask_greyL = cv2.bitwise_and(masked_bb_imgL_HSV_grey,masked_bb_imgL_HSV_grey, RedmaskL)
                      #redMask_greyR = cv2.bitwise_and(masked_bb_imgR_HSV_grey,masked_bb_imgR_HSV_grey, RedmaskR)

                      bothRedMasks = cv2.bitwise_or(RedmaskL,RedmaskR)
                      # MOMENTS-2D
                      rollLR = []
                      greenMSKS = [GreenmaskL,GreenmaskR]
                      for num,i in enumerate([RedmaskL,RedmaskR]):
                         
                        M1 = cv2.moments(i)
                        # Calculate the x and y coordinates of the centroid of the first binary mask
                        cx1 = int(M1['m10'] / M1['m00'])
                        cy1 = int(M1['m01'] / M1['m00'])
                        # Calculate moments of the second binary mask
                        M2 = cv2.moments(greenMSKS[num])
                        # Calculate the x and y coordinates of the centroid of the second binary mask
                        cx2 = int(M2['m10'] / (M2['m00']+0.000001))
                        cy2 = int(M2['m01'] /( M2['m00']+0.0000001))
                        roll =math.degrees(math.atan2((cx1-cx2),(cy2-cy1)))-180
                        rollLR.append(roll)

                      print('AVERAGE ROLL: ',sum(rollLR)/2)
                      #x = ((u - mtx_left[1][2])) * depth / mtx_left[1][1]
                      #y = -((v - mtx_left[0][2])) * depth / mtx_left[0][0]
                      x = ((u - mtx_left[1][2])) * depth /( 2*mtx_left[0][0])
                      y = -((v - mtx_left[0][2])) * depth /( mtx_left[1][1]) *1.5
                      z = depth
                      #if z >= 0:
                      data = str(z*10)+' '+str(x*10)+' '+str(y*10)
                      
                      # Bounding box
                      start_point=[boundBoxL[0],boundBoxL[3]]
                      end_point=[boundBoxL[2],boundBoxL[1]]
                      cv2.rectangle(frame_left,start_point,end_point,(0,255,0),2)
                      #print(data)
                      #cv2.putText(frame_left, f'{z}mm',(start_point[0],start_point[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                      cv2.putText(frame_left, f'{round(x,3),round(y,3),round(z,3)}x,y,z',(start_point[0],start_point[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),1)
                      await ws.send(data)
                      print('data loop: ',data)
            
            print('data: ',data)
            #if len(data) == 0:
               #data = '0 0 0'
            ##else:
              # data=data[0]
            #await ws.send(data) #send top of list of setection if there are multiple
              #end1 = time.time()

              #totalTime = end - start1
              #fps = 1 / totalTime
              #cv2.putText(frame_right, f'FPS: {int(totalTime)}', (20,450), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0), 2)


              
            # Show the frames
            cv2.imshow("frame right", frame_right) 
            cv2.imshow("frame left", frame_left)
            # Hit "q" to close the window
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            #await ws.send(data[0])
            #print(await ws.recv())

    cap.release()
    cv2.destroyAllWindows()

asyncio.get_event_loop().run_until_complete(dataSender())
asyncio.get_event_loop().run_forever()