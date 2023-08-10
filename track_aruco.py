#!/usr/bin/python

##########IMPORTS##########

import rospy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import sys
import time
import math
import numpy as np
import ros_numpy as rnp

##########VARIABLES##########

newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

id_to_find = 72 ##arucoID
marker_size = 20 ##CM

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = 640
vertical_res = 480

horizontal_fov = 62.2 * (math.pi / 180) ##62.2 for picam V2, 53.5 for V1
vertical_fov = 48.8 * (math.pi / 180) ##48.8 for V2, 41.41 for V1

found_count = 0
notfound_count = 0

time_last = 0
time_to_wait = .1
############CAMERA INTRINSICS#############

dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix = [[1061.6538553425996, 0.0, 640.5],[0.0, 1061.6538553425996, 360.5],[0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

############FUNCTIONS#############

def msg_receiver(message):
    global notfound_count, found_count, time_last, time_to_wait, id_to_find

    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(message) ##Deserialize image data into array
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)

        try:
            if ids is not None:
                if ids[0]==id_to_find:
                    ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff)
                    (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
                    x = '{:.2f}'.format(tvec[0])
                    y = '{:.2f}'.format(tvec[1])
                    z = '{:.2f}'.format(tvec[2])

                    marker_position = 'MARKER POSITION: x='+x+' y='+y+' z='+z

                    aruco.drawDetectedMarkers(np_data, corners)
                    cv2.drawFrameAxes(np_data, np_camera_matrix, np_dist_coeff, rvec, tvec, 10)

                    cv2.putText(np_data, marker_position, (10,50), 0, 1, (255,0,0), thickness=2)

                    found_count = found_count + 1
                
                else:
                    notfound_count = notfound_count + 1
                
            else:
                notfound_count = notfound_count + 1

        except Exception as e:
            print('Target likely not found!')
            print(e)
            notfound_count = notfound_count + 1
        new_msg = rnp.msgify(Image, np_data, encoding='rgb8')
        newimg_pub.publish(new_msg)
        time_last = time.time()
    else:
        return None
                

def subscriber():
    rospy.init_node('drone_node', anonymous=False)
    sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)
    rospy.spin()

if __name__=='__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
