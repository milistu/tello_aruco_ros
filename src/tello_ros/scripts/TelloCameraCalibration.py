#!/usr/bin/env python3
from djitellopy import Tello
import cv2
import numpy as np
import glob
import rospkg
import json
import os


################ CODE FOR TAKING IMAGES FROM TELLO VIDEO FEED ################

####################################################
width = 640 #320 # WIDTH OF THE IMAGE
height = 480 #240 # HEIGHT OF THE IMAGE
startCounter = 1 # 0 FOR FLIGHT 1 FOR TESTING
####################################################

# Getting paths
rospack = rospkg.RosPack()
imgPath = rospack.get_path('tello_ros')
cameraCalibPath = imgPath + "/cameraCalibration/"
imgPath = imgPath + "/cameraCalibration/images/"

# True = whole proces will be done
# False = if you already taken the images from Tello and you wan't to run the calibration again
if True:

    # CONNECT TO TELLO
    me = Tello()
    me.connect()
    me.for_back_velocity = 0 # Forward/backward
    me.left_right_velocity = 0
    me.up_down_velocity = 0
    me.yaw_velocity = 0
    me.speed = 0

    print("Tello battery status: ", me.get_battery())
    print("Tello temperature status: ", me.get_temperature())

    me.streamoff()
    me.streamon()
    i = 0

    # If you want more image for camera calibration change the number below, default is 4 image
    while i < 8:
        # GET THE IMAGE FROM TELLO
        frame_read = me.get_frame_read()
        myFrame = frame_read.frame
        img = cv2.resize(myFrame, (width, height))
        cv2.imshow("Video feed from TELLO", img)

        # TO GO UP IN THE BEGINNING
        if startCounter == 0:
            me.takeoff()
            me.move_left(20)
            me.rotate_clockwise(90)
            startCounter = 1

        if cv2.waitKey(1) & 0xFF == ord('s'): # s - for save the image
            cv2.imwrite(imgPath + "calibImg" + str(i + 1) + ".jpg", img)
            print(" Image taken ", i+1)
            i += 1

    cv2.destroyAllWindows()
    # Landing the drone if its flying
    if startCounter == 0:
        me.land()
    # Ending the CONNECTION with Tello
    me.end()

print("\n\n\n<<<<<<<<<< PLEASE WAIT FOR CAMERA CALIBRATION TO BE COMPLETED, THANK YOU FOR YOUR PATIENCE! >>>>>>>>>>\n\n\n")

################ CODE FOR CAMERA CALIBRATION ################

################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

# IF YOU ARE NOT USING THE CHESSBOARD PROVIDED IN DOCUMENTATION YOU NEED TO CHANGE BELOW PARAMETERS: chessboardSize, size_of_chessboard_squares_mm (CHECK THIS PARAMETER EVEN IF YOU USE OUR CHESSBOARD)
chessboardSize = (9,7)
frameSize = (width,height)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

size_of_chessboard_squares_mm = 21
objp = objp * size_of_chessboard_squares_mm


# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob(imgPath + '*.jpg')

i = 0;
for image in images:

    img = cv2.imread(image)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)

    # If found, add object points, image points (after refining them)
    if ret == True:

        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, chessboardSize, corners2, ret)
        #cv2.imshow('Image', img)
        cv2.imshow('Image '+str(i+1), img)
        cv2.waitKey(1000)
        cv2.destroyAllWindows()
        i +=1


cv2.destroyAllWindows()

############## CALIBRATION #######################################################

ret, cameraMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frameSize, None, None)

f= open(cameraCalibPath + "CameraCalibrationParameters.txt","w+")

print("\n\nCamera Matrix:\n", cameraMatrix)
f.write("Camera Matrix:\n ")
f.write(str(cameraMatrix))
print("\n\nDistortion Paramerets:\n", dist)
f.write("\n\nDistortion Paramerets:\n")
f.write(str(dist))

# Calibration data to be written as .json
cv_file = cv2.FileStorage(cameraCalibPath + "CamCalibParam.json", cv2.FILE_STORAGE_WRITE)
cv_file.write("Camera_Matrix", cameraMatrix)
cv_file.write("Distortion_Parameters", dist)
cv_file.release()

# Reprojection Error
mean_error = 0

for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error
totalError = mean_error/len(objpoints)
print( "\nTotal error: {}".format(totalError))
f.close()
