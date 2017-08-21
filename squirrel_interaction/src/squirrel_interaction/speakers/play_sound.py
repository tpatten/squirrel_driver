#!/usr/bin/env python

"""

This ROS Node provides an interface for the Speakers connected to the Raspberry Pi on project SQUIRREL.

It makes use of the aplay application to do so.

The service /sound/play is called with the name of the soundfile to be played.

"""

import rospy
import os
#from play_sound.srv import *
import actionlib
from sound_play.msg import *

def play(data):
    """ Plays the sound file with the name passed as parameter """
    comand='aplay -q /home/pi/catkin_ws/src/play_sound/sounds/"'+data.message+'"'
    print comand
    os.system(comand)
    return True


def listener():
    rospy.init_node('playsound')
    rospy.Service('sound/play', PlaySound, play)
    rospy.loginfo("Ready to Play Sounds")
    rospy.spin()

if __name__ == '__main__':
    listener()
