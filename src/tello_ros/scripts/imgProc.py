#!/usr/bin/env python3

import rospy
import rospkg
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

frameWidth = 640
frameHeight = 480
deadZone = 80 #100

global imgContour
global dir
dir = 0

### TELLO camera parameters

# Getting paths
rospack = rospkg.RosPack()
cameraCalibPath = rospack.get_path('tello_ros')
cameraCalibPath = cameraCalibPath + "/cameraCalibration/"


# Read json file
cv_file = cv2.FileStorage(cameraCalibPath + "CamCalibParam.json", cv2.FILE_STORAGE_READ)

# Distortion Paramerets:
dist = cv_file.getNode("Distortion_Parameters").mat()
# Camera Matrix:
mtx = cv_file.getNode("Camera_Matrix").mat()

# Dimensions of ArUco tag in m, we are using 50x50mm tag
# If you are using tag with different dimension you need to change this:
tagDim = np.array([[0.025,  -0.025, 1],
        [0.025, 0.025, 1],
        [-0.025, 0.025, 1],
        [-0.025, -0.025, 1]])

# For expanding corners array
expander = np.array([[1, 1, 1, 1]])
# Font for displaying text (below)
font = cv2.FONT_HERSHEY_COMPLEX
fontColor = (255, 0, 0) # bgr = BLUE

# Display the grid
def dispay(img):
    frameHeight = img.shape[0]
    frameWidth = img.shape[1]

    cv2.line(img, (int(frameWidth/2)-deadZone, 0), (int(frameWidth/2)-deadZone, frameHeight), (255, 255, 0), 3)
    cv2.line(img, (int(frameWidth / 2) + deadZone, 0), (int(frameWidth / 2) + deadZone, frameHeight), (255, 255, 0), 3)

    cv2.circle(img, (int(frameWidth/2), int(frameHeight/2)), 5, (0, 0, 255), 5)
    cv2.line(img, (0, int(frameHeight / 2) - deadZone), (frameWidth, int(frameHeight / 2) - deadZone), (255, 255, 0), 3)
    cv2.line(img, (0, int(frameHeight / 2) + deadZone), (frameWidth, int(frameHeight / 2) + deadZone), (255, 255, 0), 3)

    return img


def centerOfmarker(corners):
    k1 = (corners[0][0][0][1] - corners[0][0][2][1])/(corners[0][0][0][0] - corners[0][0][2][0])
    k2 = (corners[0][0][3][1] - corners[0][0][1][1])/(corners[0][0][3][0] - corners[0][0][1][0])
    b1 = -corners[0][0][2][0]*k1 + corners[0][0][2][1]
    b2 = -corners[0][0][1][0]*k2 + corners[0][0][1][1]
    centerX = (b2-b1)/(k1-k2)
    centerY = k1*centerX + b1

    return int(centerX), int(centerY)
global mover
mover = 15
global circleRadius
circleRadius = 70
def pilotControl(imgContour, cx, cy, transform_translation_z, ids):
    global dir
    global mover
    global circleRadius
    frameHeight = imgContour.shape[0]
    frameWidth = imgContour.shape[1]

    if mover > 45:
        mover = 15
    if circleRadius < 35 or circleRadius > 105:
        circleRadius = 70
    if (ids[0] == 10):
        cv2.putText(imgContour, " LANDING ", (-15, 80), font, 0.9, fontColor, 3)
        dir = 13
    elif (transform_translation_z > 0.6 ):
        cv2.putText(imgContour, " MOVING ", (-15, 80), font, 0.9, fontColor, 3)
        cv2.putText(imgContour, " FORWARD ", (-15, 110), font, 0.9, fontColor, 3)
        cv2.circle(imgContour, (int(frameWidth/2), int(frameHeight/2)), circleRadius - 35, (0, 255, 0), 5)
        cv2.circle(imgContour, (int(frameWidth/2), int(frameHeight/2)), 20, (0, 255, 0), -1)
        circleRadius += 1
        # cv2.rectangle(imgContour, (80*2, int(frameHeight / 2 - deadZone)),((int(frameWidth / 2) - deadZone) + 80*2, int(frameHeight / 2) + deadZone), (255, 0, 0), cv2.FILLED)
        dir = 10 # GO FORWARD
    elif (transform_translation_z < 0.4):
        cv2.putText(imgContour, " MOVING ", (-15, 80), font, 0.9, fontColor, 3)
        cv2.putText(imgContour, " BACKWARD ", (-15, 110), font, 0.9, fontColor, 3)
        cv2.circle(imgContour, (int(frameWidth/2), int(frameHeight/2)), circleRadius, (0, 255, 0), 5)
        dot = int(circleRadius / np.sqrt(2))
        cv2.line(imgContour, (int(frameWidth/2) - dot, int(frameHeight/2) - dot), (int(frameWidth/2) + dot, int(frameHeight/2) + dot), (0, 255, 0), 8)
        cv2.line(imgContour, (int(frameWidth/2) + dot, int(frameHeight/2) - dot), (int(frameWidth/2) - dot, int(frameHeight/2) + dot), (0, 255, 0), 8)
        circleRadius -= 1
        # cv2.rectangle(imgContour, (80*2, int(frameHeight / 2 - deadZone)),((int(frameWidth / 2) - deadZone) + 80*2, int(frameHeight / 2) + deadZone), (0, 0, 255), cv2.FILLED)
        dir = 11 # GO BACKWARD
    elif (cx < int(frameWidth / 2) - deadZone):
        cv2.putText(imgContour, " TURNING ", (-15, 80), font, 0.9, fontColor, 3)
        cv2.putText(imgContour, " LEFT ", (-15, 110), font, 0.9, fontColor, 3)
        # define the arrow shape
        arrowHorizontalLeft = np.array([[[0, 0], [50, 50], [50, 20], [100, 20],
                                         [100, -20], [50, -20], [50, -50]]])

        # move it to the desired position
        cxArrow = int(frameWidth / 2) - 80 - 100 - mover
        mover += 1
        cyArrow = int(frameHeight / 2)
        arrowHorizontalLeft[:, :, 0] += cxArrow
        arrowHorizontalLeft[:, :, 1] += cyArrow
        cv2.drawContours(imgContour, arrowHorizontalLeft, -1, (0, 255, 0), -1)
        # cv2.rectangle(imgContour, (0, int(frameHeight / 2 - deadZone)),(int(frameWidth / 2) - deadZone, int(frameHeight / 2) + deadZone), (0, 0, 255), cv2.FILLED)
        dir = 1
    elif (cx > int(frameWidth / 2) + deadZone):
        cv2.putText(imgContour, " TURNING ", (-15, 80), font, 0.9, fontColor, 3)
        cv2.putText(imgContour, " RIGHT ", (-15, 110), font, 0.9, fontColor, 3)
        # define the arrow shape
        arrowHorizontalRight = np.array([[[0, 0], [-50, -50], [-50, -20], [-100, -20],
                                          [-100, 20], [-50, 20], [-50, 50]]])

        # move it to the desired position
        cxArrow = int(frameWidth / 2) + 80 + 100 + mover
        mover += 1
        cyArrow = int(frameHeight / 2)
        arrowHorizontalRight[:, :, 0] += cxArrow
        arrowHorizontalRight[:, :, 1] += cyArrow
        cv2.drawContours(imgContour, arrowHorizontalRight, -1, (0, 255, 0), -1)
        # cv2.rectangle(imgContour, (int(frameWidth / 2 + deadZone), int(frameHeight / 2 - deadZone)),(frameWidth, int(frameHeight / 2) + deadZone), (0, 0, 255), cv2.FILLED)
        dir = 2
    elif (cy < int(frameHeight / 2) - deadZone):
        cv2.putText(imgContour, " GOING ", (-15, 80), font, 0.9, fontColor, 3)
        cv2.putText(imgContour, " UP ", (-15, 110), font, 0.9, fontColor, 3)
        # define the arrow shape
        arrowVerticalUp = np.array([[[0, 0], [50, 50], [20, 50], [20, 100],
                                     [-20, 100], [-20, 50], [-50, 50]]])

        # move it to the desired position
        cxArrow = int(frameWidth / 2)
        cyArrow = int(frameHeight / 2) - 80 - 100 - mover
        mover += 1
        arrowVerticalUp[:, :, 0] += cxArrow
        arrowVerticalUp[:, :, 1] += cyArrow
        cv2.drawContours(imgContour, arrowVerticalUp, -1, (0, 255, 0), -1)
        # cv2.rectangle(imgContour, (int(frameWidth / 2 - deadZone), 0),(int(frameWidth / 2 + deadZone), int(frameHeight / 2) - deadZone), (0, 0, 255), cv2.FILLED)
        dir = 3
    elif (cy > int(frameHeight / 2) + deadZone):
        cv2.putText(imgContour, " GOING ", (-15, 80), font, 0.9, fontColor, 3)
        cv2.putText(imgContour, " DOWN ", (-15, 110), font, 0.9, fontColor, 3)
        # define the arrow shape
        arrowVerticalDown = np.array([[[0, 0], [-50, -50], [-20, -50], [-20, -100],
                                       [20, -100], [20, -50], [50, -50]]])

        # move it to the desired position
        cxArrow = int(frameWidth / 2)
        cyArrow = int(frameHeight / 2) + 80 + 100 + mover
        mover += 1
        arrowVerticalDown[:, :, 0] += cxArrow
        arrowVerticalDown[:, :, 1] += cyArrow
        cv2.drawContours(imgContour, arrowVerticalDown, -1, (0, 255, 0), -1)
        # cv2.rectangle(imgContour, (int(frameWidth / 2 - deadZone), int(frameHeight / 2) + deadZone),(int(frameWidth / 2 + deadZone), frameHeight), (0, 0, 255), cv2.FILLED)
        dir = 4
    else:
        dir = 0
        circleRadius = 70
        mover = 15

    cv2.line(imgContour, (int(frameWidth / 2), int(frameHeight / 2)), (cx, cy), (0, 0, 255), 3)

    return imgContour

def MyPoseEstimationOfSingleMarker(corners):
    cornersImg = np.concatenate((corners, expander.T), axis=1)
    cornersImg = np.matrix.transpose(cornersImg)

    H = np.dot(np.dot(cornersImg, np.matrix.transpose(tagDim.T)), np.linalg.inv(np.dot(tagDim.T, np.matrix.transpose(tagDim.T))))

    X = np.dot(np.linalg.inv(mtx), H)
    # Mislim da ovde treba dodati i za r2 - MILUTIN
    s = np.linalg.norm(X[:, 0])

    X_div = np.divide(X, s)

    distance = X_div[:, 2]

    return distance

def processing(img):

    img = cv2.resize(img, (frameWidth, frameHeight))
    h1, w1 = img.shape[:2]
    # Read the camera picture
    # Correct distortion
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (h1, w1), 0, (h1, w1))
    dst1 = cv2.undistort(img, mtx, dist, None, newcameramtx)
    x, y, w1, h1 = roi
    dst1 = dst1[y:y + h1, x:x + w1]
    img=dst1


    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters =  cv2.aruco.DetectorParameters_create()
    # dst1 = cv2.undistort(img, mtx, dist, None, newcameramtx)

    # Use aruco The detectmarkers() function can detect the marker and return the ID and the coordinates of the four corners of the sign board
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray,aruco_dict,parameters=parameters)

    # If you can't find it, type id
    if ids is not None:

        #rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist) # treba da budu nule jer je prethodno undistortovano proveri tacnost
        #ovu funkciju izbaci, izracnaj na osnovu onoga sa casa , coskove -> matrica homografije -> poziciju i orjentaciju 
        tvec = MyPoseEstimationOfSingleMarker(corners[0][0])
        # Estimate the attitude of each marker and return the values rvet and tvec --- different
        # from camera coeficcients
        # (rvec-tvec).any() # get rid of that nasty numpy value array error

        # aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.1) #Draw axis
        # aruco.drawDetectedMarkers(frame, corners) #Draw a square around the mark

        for i in range(3): # rvec.shape[0]

            #cv2.drawFrameAxes(img, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.03)
            cv2.aruco.drawDetectedMarkers(img, corners)
        ###### DRAW ID #####
        cv2.putText(img, "Id: " + str(ids), (0,30), font, 1, (0,255,0),2,cv2.LINE_AA)

        centerX, centerY = centerOfmarker(corners)
        cv2.circle(img, (int(centerX),int(centerY)), 4, (255, 0, 255), 2)

        transform_translation_z = tvec[2]
        img = pilotControl(img, centerX, centerY, transform_translation_z, ids)

    else:
        ##### DRAW "NO IDS" #####
        cv2.putText(img, "No Ids", (0,30), font, 1, (0,255,0),2,cv2.LINE_AA)

    img = dispay(img)
    return img

class Nodo(object):

    def __init__(self):
        # Params
        self.imageReceived = None
        self.imageToPub = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(30)
        # Publishers
        self.pub = rospy.Publisher('imageProcessed', Image,queue_size=10)
        self.pubComm = rospy.Publisher('tello_commands', Int8, queue_size=10)
        # Subscribers
        rospy.Subscriber("video_frames_TELLO", Image, self.callback)

    def callback(self, msg):

        rospy.loginfo('Image received...')
        # Transfering received image from ROS to cv2 type
        self.imageReceived = self.br.imgmsg_to_cv2(msg)
        self.imageToPub = processing(self.imageReceived)

    def start(self):

        global dir
        while not rospy.is_shutdown():
            rospy.loginfo('publishing commands')
            self.pubComm.publish(dir)
            # When dir published -> refresh
            dir = 0
            if self.imageToPub is not None:
                # Publishing processed images
                rospy.loginfo('publishing processed video frame')
                self.pub.publish(self.br.cv2_to_imgmsg(self.imageToPub))
            self.loop_rate.sleep()

if __name__ == '__main__':
    # Initialization
    rospy.init_node("imageSubPub", anonymous=True, disable_signals=True)
    my_node = Nodo()
    my_node.start()
