import cv2 
import glob
import numpy as np


stoppingCrtieria= (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
chessboardDims=(11,7)
frameSize=(640,480)  # important as it will affect the usage of parameters at different resolutions

#Object points, based on calibration pattern and dims
objectPoints=[]
objPts =  np.zeros((chessboardDims[0]*chessboardDims[1],3),np.float32)
objPts[:,:2] = np.mgrid[0:chessboardDims[0],0:chessboardDims[1]].T.reshape(-1,2)
#objPts=objPts*25
leftImagePoints = []
rightImagePoints = []

# Loading calibration images
leftImgs=glob.glob('stereoImgs\imgL\*.png')
rightImgs=glob.glob('stereoImgs\imgR\*.png')

#Find chessboard corners
for imgL, imgR in zip(leftImgs,rightImgs):
    leftImg=cv2.imread(imgL)
    rightImg=cv2.imread(imgR)
    grayLeft=cv2.cvtColor(leftImg,cv2.COLOR_BGR2GRAY)
    grayRight=cv2.cvtColor(rightImg,cv2.COLOR_BGR2GRAY)
    #Detect chessboards
    leftRet, leftCorners = cv2.findChessboardCorners(grayLeft, chessboardDims)
    rightRet, rightCorners = cv2.findChessboardCorners(grayRight, chessboardDims)

    if leftRet and rightRet == True:
        objectPoints.append(objPts)
        cornersL = cv2.cornerSubPix(grayLeft, cornersL, (11,11), (-1,-1), stoppingCrtieria)
        leftImagePoints.append(cornersL)
        cornersR = cv2.cornerSubPix(grayRight, cornersR, (11,11), (-1,-1), stoppingCrtieria)
        rightImagePoints.append(cornersR)


    
#LHS-calibraton
leftHeight, leftWidth,leftChannels = leftImg.shape
leftRet, leftCameraMatrix, leftDist, left_rvecs, left_tvecs= cv2.calibrateCamera(objectPoints, leftImagePoints, frameSize, None, None)
leftNewCameraMatrix, leftRoi = cv2.getOptimalNewCameraMatrix(leftCameraMatrix, leftDist, (leftWidth, leftHeight), 1, (leftWidth, leftHeight))

#RHS-calibration
rightHeight, rightWidth, rightChannels = rightImg.shape
rightRet, rightRameraMatrix, rightDist, right_rvecs, right_tvecs = cv2.calibrateCamera(objectPoints, rightImagePoints, frameSize, None, None)
rightNewCameraMatrix, leftRoi = cv2.getOptimalNewCameraMatrix(leftCameraMatrix, rightDist, (rightWidth, rightWidth), 1, (rightWidth,rightHeight))

# stereo calibrate to find extrinsics
ret, _, _,_, _, R, T,_,_ = cv2.stereoCalibrate(objectPoints, leftImagePoints, rightImagePoints, leftNewCameraMatrix, leftDist,
                                                                 rightNewCameraMatrix, leftDist, (leftWidth, leftHeight), stoppingCrtieria, flags = cv2.CALIB_FIX_INTRINSIC)

print('#####-Intrinsics-#####')
print('M_L',leftNewCameraMatrix)
print('M_R',rightNewCameraMatrix)
print('dist_L',leftDist)
print('dist_R',rightDist)


print('#####-Extrinsics-#####')
print('R: ',R)
print('T: ',T)
print('RMSE: ',ret)

