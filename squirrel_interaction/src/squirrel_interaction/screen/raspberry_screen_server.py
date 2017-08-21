#!/usr/bin/env python

"""

This ROS Node provides an interface for the Monitor attached to the Raspberry Pi on project SQUIRREL.

It makes use of the Eye of Gnome (EOG)  application to do so.

The service /face/emotion is called with the name of the image to be displayed.

"""

import rospy, rospkg
import os,sys
from squirrel_interaction.srv import DisplayScreen

image=None
rospack = rospkg.RosPack()
path = rospack.get_path('squirrel_interaction')


def handle_display_on_screen(data):
    global image,rospack,path
    print "Emotion: "+data.message
    try:
        if(data.message=="stop"):
            os.system("kill $(pidof eog) &")
            rospy.signal_shutdown('Bye')
        elif data.message == "blank":
            to_open = "%s/resources/media/images/blank.png" % path
            os.system('eog --fullscreen -w \"'+to_open+'\" &')
        #elif data.message == "video":
        #    to_open = "%s/resources/media/videos/video1.mp4" % path
        #    os.system("vlc --fullscreen --quiet %s &" % to_open)
        else:
            to_open = "%s/resources/media/images/%s.png" % (path,data.message)
            if not os.path.isfile(to_open):
                return {'result':'No such file'}
            elif(image != data.message):
                os.system('eog --fullscreen -w "'+to_open+'" &')
        image=data.message
        return {'result':True}

    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
        return {'result':False}



def main():
    rospy.init_node('raspberry_screen_server')
    s = rospy.Service('face/emotion', DisplayScreen, handle_display_on_screen)
    rospy.loginfo("Ready to display on screen")
    rospy.spin()

if __name__ == "__main__":
    main()
