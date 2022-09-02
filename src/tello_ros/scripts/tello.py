#!/usr/bin/env python3

from djitellopy import Tello
import rospy 
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import cv2

frameWidth = 640
frameHeight = 480

# 0 = FLIGHT ACTIVATED
# 1 = FLIGHT DEACTIVATED = DEFAULT
startCounter = 0

# Connect to TELLO
me = Tello()
me.connect()
me.for_back_velocity = 0 # Forward/backward
me.left_right_velocity = 0
me.up_down_velocity = 0
me.yaw_velocity = 0
me.speed = 0

print("Tello BATTERY: ", me.get_battery())
print("Tello TEMPERATURE: ", me.get_temperature())

me.streamoff()
me.streamon()

def exeComm(msg):
    rospy.loginfo('Command received...')
    # print(msg.data)

    if msg.data == 13:
        # Tag with ID 10 detected -> landing
        me.land()
    elif msg.data == 10:
        # Drone out of boundary (distance greater than 0.55 meters) -> moving forward
        me.for_back_velocity = 20
    elif msg.data == 11:
        # Drone out of boundary (distance less than 0.4 meters) -> moving backward
        me.for_back_velocity = -20
    elif msg.data == 1:
        # Turning left
        me.yaw_velocity = -20
    elif msg.data == 2:
        # Turning right
        me.yaw_velocity = 20
    elif msg.data == 3:
        # Going up
        me.up_down_velocity = 20
    elif msg.data == 4:
        # Going down
        me.up_down_velocity = -20
    else:
        # Refresh
        me.left_right_velocity = 0; me.for_back_velocity = 0; me.up_down_velocity = 0; me.yaw_velocity = 0
    # SEND VELOCITY VALUES TO TELLO
    if me.send_rc_control:
        me.send_rc_control(me.left_right_velocity, me.for_back_velocity, me.up_down_velocity, me.yaw_velocity)
    msg.data = 0

def pubSubNode():
    # Publishers
    pub = rospy.Publisher('video_frames_TELLO', Image, queue_size=10)
    # Subscribers
    rospy.Subscriber("tello_commands", Int8, exeComm)
    # Initialization
    rospy.init_node('TELLO_video_pub_py', anonymous=True, disable_signals=True)

    rate = rospy.Rate(30)

    br = CvBridge()

    while not rospy.is_shutdown():
        # Get video fram from Tello
        frame_read = me.get_frame_read()
        myFrame = frame_read.frame
        img = cv2.resize(myFrame, (frameWidth, frameHeight))

        rospy.loginfo('publishing TELLO video frame')
        pub.publish(br.cv2_to_imgmsg(img, 'bgr8'))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("LANDING")
            me.land()
        
        rate.sleep()

if __name__ == '__main__':
    try:
        ### FLIGHT
        if startCounter == 0:
            me.takeoff()
            startCounter = 1
        pubSubNode()
    except rospy.ROSInterruptException:
        print("LANDING")
        pass
