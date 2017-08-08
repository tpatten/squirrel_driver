#!/usr/bin/env python
"""
    This ROS Node provides an interface for the Speakers connected to the Raspberry Pi on project SQUIRREL.

    It makes use of the aplay application to do so.

    The service /sound/play is called with the name of the soundfile to be played.
"""

import rospy
import os
from squirrel_interaction.srv import PlaySound
import actionlib

def play(data):
    current_dir_name = os.path.dirname(os.path.realpath(__file__))
    comand='aplay -q ' + current_dir_name + '../../../resources/sounds/"'+data.message+'"'
    print comand
    os.system(comand)
    return True

#def speak(data):
    #DISABLED TEMPORARILY
    #goal = SoundRequest(sound=-3, command=1, volume=1.0, arg='data')
    #client.send_goal(goal)
#   return

def listener():
    rospy.init_node('playsound')
    rospy.Service('/sound/play', PlaySound, play)
    rospy.loginfo("Ready to Speak");
    rospy.spin()

if __name__ == '__main__':
    listener()
