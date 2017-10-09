#!/usr/bin/env python

"""

This ROS Node provides an interface for the Monitor attached to the Raspberry Pi.
<<<<<<< HEAD
=======
All .png files under $(find squirrel_interaction)/resources/media/images are available.
>>>>>>> 8f4f13e531814dcf83efcfb69a3e46804c58860b

"""

<<<<<<< HEAD
The service /face/emotion is called with the name of the image to be displayed without extention.

All .png files under $(find squirrel_interaction)/resources/media/images are available.

"""

import os
import rospy
import rospkg
from squirrel_interaction.srv import *

ROSPACK = rospkg.RosPack()
PATH = ROSPACK.get_path('squirrel_interaction')

def handle_display_on_screen(data):
    """ If message passed to service is "stop", kill node and EOG.

        If file exists, displays it.
    """
    try:
        base_color = rospy.get_param("face_color", default="navy")
        if data.message == "stop":
            os.system("kill $(pidof eog) &")
            rospy.signal_shutdown('EOG application node closed. Reason: User Command')
        else:
            to_open = "%s/resources/media/images/%s/%s.png" % (PATH, base_color, data.message)
            os.system('eog --fullscreen -w "' + to_open + '" &')
        return True

    except rospy.ServiceException as exc:
        print "Service did not process request: " + str(exc)
        return False

def get_available_expressions(data):
    """ lists all face names
    """
    base_color = rospy.get_param("face_color", default="navy")
    files_in_dir = os.listdir(PATH + "/resources/media/images/" + base_color)
    expressions = __filter_png_files(files_in_dir)
    names = __remove_extensions(expressions)
    return GetListResponse(names)

=======
import os
import cv2
import rospy
import rospkg
from squirrel_interaction.srv import *
from std_msgs.msg import String

ROSPACK = rospkg.RosPack()
PATH = ROSPACK.get_path('squirrel_interaction')
BASE_COLOR = rospy.get_param("face_color", default="navy")

def emotion_cb(msg):
    global img_file
    base_color = rospy.get_param("face_color", default="gold")
    img_file= "%s/resources/media/images/%s/%s.png" % (PATH, base_color, msg.data)

def get_available_expressions(data):
    """ lists all face names
    """
    base_color = rospy.get_param("face_color", default="navy")
    files_in_dir = os.listdir(PATH + "/resources/media/images/" + base_color)

    expressions = __filter_png_files(files_in_dir)
    names = __remove_extensions(expressions)
    return GetListResponse(names)

>>>>>>> 8f4f13e531814dcf83efcfb69a3e46804c58860b
def __filter_png_files(files):
    for curr_file in files:
        if curr_file.endswith(".png"):
            yield curr_file

def __remove_extensions(files):
    return [filename[:-4] for filename in files]


def main():
    global img_file
    rospy.init_node('raspberry_screen_server')
<<<<<<< HEAD
    rospy.Service('face/emotion', DisplayScreen, handle_display_on_screen)
    rospy.Service('face/emotions', GetList, get_available_expressions)
=======
    rospy.Subscriber("/face/emotion", String, emotion_cb)
    rospy.Service('face/emotions', GetList, get_available_expressions)
    cv2.namedWindow("image", cv2.WND_PROP_FULLSCREEN)          
    cv2.setWindowProperty("image", cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)

    img_file= "{}/resources/media/images/{}/{}.png".format(PATH, "gold", "look_down")
>>>>>>> 8f4f13e531814dcf83efcfb69a3e46804c58860b
    rospy.loginfo("Ready to display on screen")

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        image = cv2.imread(img_file)
        cv2.imshow('image', image)
        cv2.waitKey(20)
        r.sleep()
    rospy.spin()


if __name__ == "__main__":
    main()
