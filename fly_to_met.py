import rospy
import cv2
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from math import sqrt
import numpy
import math
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('computer_vision_sample')
bridge = CvBridge()

image_pub_mask = rospy.Publisher('blur_mask', Image)

image_pub = rospy.Publisher('CONT', Image)

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def uderz():
    telemetry = get_telemetry(frame_id='aruco_map')
    navigate_wait(x=telemetry.x, y=telemetry.y, z=1, yaw=np.pi, frame_id='aruco_map')

def make_vector(p1, p2):
    return p2[0] - p1[0], p2[1] - p1[1]
 
def get_len(v):
    return math.sqrt(v[0] ** 2 + v[1] ** 2)
 
def get_angle(v1, v2):
    return math.acos(
        (v1[0] * v2[0] + v1[1] * v2[1]) /
        (get_len(v1) * get_len(v2)))

def obvod(cord):
    frame = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    frameCop = frame.copy()
    colors = {'blue': [25, 150, 150, 35, 255, 255, [255, 255, 0]]}

    colors_name = ['blue']
    for i in range(len(colors_name)):
        hsv_color = colors[colors_name[i]]
        hsv_min = np.array((hsv_color[0], hsv_color[1], hsv_color[2]), np.uint8)
        hsv_max = np.array((hsv_color[3], hsv_color[4], hsv_color[5]), np.uint8)

        hsv = cv.cvtColor(frame.copy(), cv.COLOR_BGR2HSV)
        hsv = cv2.blur(hsv, (5, 5))

        obrab_hsv = cv2.inRange(hsv, hsv_min, hsv_max)
        obrab_hsv = cv2.erode(obrab_hsv, None, iterations = 2)
        obrab_hsv = cv2.dilate(obrab_hsv, None, iterations = 4)

        image_pub_mask.publish(bridge.cv2_to_imgmsg(obrab_hsv, 'mono8'))

        none, conturs, none2 = cv.findContours(obrab_hsv.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
        if conturs:
            conturs = sorted(conturs, key=cv.contourArea, reverse=True)

            (x, y, w, h) = cv.boundingRect(conturs[0])
            cv.rectangle(frame, (x, y), (x + w, y + h), (hsv_color[6][0], hsv_color[6][1], hsv_color[6][2]), 2)

            cent_obect_x = x + w/2
            cent_obect_y = y + h/2

            x_y_cent_frame = 160, 120
            cv2.circle(frame, (cent_obect_x, cent_obect_y), 5, (0, 0, 255), thickness=-1)
            if cord == 'x':
                rast = abs(max(cent_obect_x, x_y_cent_frame[0]) - min(cent_obect_x, x_y_cent_frame[0]))
                if cent_obect_x < x_y_cent_frame[0]:
                    napr = 'left'
                else:
                    napr = 'right'
                return napr, rast
            if cord == 'y':
                rast = abs(max(cent_obect_y, x_y_cent_frame[1]) - min(cent_obect_y, x_y_cent_frame[1]))
                if cent_obect_y < x_y_cent_frame[1]:
                    napr = 'down'
                else:
                    napr = 'up'
                return napr, rast
                


navigate_wait(z=1, frame_id='body', auto_arm=True)
navigate_wait(x=0, y=0, z=1, yaw=np.pi, frame_id='aruco_map')

while not rospy.is_shutdown():
    napr, rast_x = obvod('x')
    if rast_x > 10:
        on_pos_x = False
        if napr == 'left':
            navigate(x=0, y=0.1, z=0, speed=1, frame_id='navigate_target')
        if napr == 'right':
            navigate(x=0, y=-0.1, z=0, speed=1, frame_id='navigate_target')
        rospy.sleep(0.2)
    else:
        on_pos_x = True
    napr, rast_y = obvod('y')
    if rast_y > 10:
        on_pos_y = False
        if napr == 'up':
            navigate(x=-0.1, y=0, z=0, speed=1, frame_id='navigate_target')
        if napr == 'down':
            navigate(x=0.1, y=0, z=0, speed=1, frame_id='navigate_target')
        rospy.sleep(0.2)
    else:
        on_pos_y = True
    if on_pos_x and on_pos_y:
        uderz()
        print('Im na metke')
    rospy.sleep(0.2)