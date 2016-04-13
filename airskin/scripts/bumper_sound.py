#!/usr/bin/env python
#
# Listens to airskin/arm_bumper and plays bump sound
#
# date: April 2016
# author: Michael Zillich (michael.zillich@tuwien.ac.at)

import rospy
import rospkg
import std_msgs.msg
from sound_play.msg import SoundRequest

sound_pub = rospy.Publisher('/robotsound', SoundRequest, queue_size=5);
rospack = rospkg.RosPack()
sound_file = rospack.get_path('airskin') + "/data/bump.wav"

def bumper_callback(msg):
    if msg.data == True:
        sound_msg = SoundRequest()
        sound_msg.sound = -2; # play file
        sound_msg.command = 1; # play once
        sound_msg.arg = sound_file;
        sound_pub.publish(sound_msg);

if __name__ == '__main__':
    rospy.init_node('bumper_sound', anonymous=True)
    rospy.Subscriber("/airskin/arm_bumper", std_msgs.msg.Bool, bumper_callback)
    rospy.spin()
