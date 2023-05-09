import websockets
import asyncio
import torch
import cv2
import time
import numpy as np

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
                  if confidence > 0.6:
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
                      #print(mtx_left)

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
                      #print('data loop: ',data)
            
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