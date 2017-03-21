#!/usr/bin/env python

# Squirrel control using Gamepad - LEDs, motors, screen and sound
# dgameiro@idmind.pt, 2017

import rospy
import sys
import numpy as np
from std_msgs.msg import String
from idmind_interaction.msg import *
from raspberry_screen.srv import DisplayScreen
from play_sound.srv import PlaySound
import thread


class SquirrelControl():
    def __init__(self):
        self.joy_sub = rospy.Subscriber("/teleop_buttons",String,self.joy_callback)
        self.pub = rospy.Publisher("idmind_interaction/leds",LedControl, queue_size=100)
        self.pub_base = rospy.Publisher("idmind_interaction/leds_base",LedControlBase,queue_size=100)
        self.pub_mouth = rospy.Publisher("idmind_interaction/leds_mouth",LedControlMouth,queue_size=100)
        rospy.sleep(0.4) #sleep for publishers to be ready
        self.playsound=rospy.ServiceProxy('playsound', PlaySound)
        self.led_config = np.zeros(126)
        self.base_config = LedControlBase()
        self.mouth_config = LedControlMouth()
        self.message = ""
        self.callback_flag = 0
        self.door_opened = False


    def joy_callback(self,msg):
        self.message = msg.data
        self.callback_flag = 1


    def face(self,message):
        rospy.wait_for_service('display_screen')
        try:
            display_image = rospy.ServiceProxy('display_screen', DisplayScreen)
            return display_image(message).result
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False


    def save_config(self,led_array):
        self.led_config = led_array


    def load_config(self):
        led_array = self.led_config
        self.base_config.header.stamp = rospy.Time.now()
        self.base_config.led_array = led_array.tolist()
        self.pub_base.publish(self.base_config)
        rospy.sleep(0.03)


    def reset_leds(self):
        config = LedControl()
        for i in range(2):
            config.header.stamp = rospy.Time.now()
            config.device = i
            config.r = 0
            config.g = 0
            config.b = 0
            self.pub.publish(config)


    def default_face(self):
        return self.face("look_front")


    # Blue 407nm - R 0 G 153 B 255
    def default_leds(self):
        config = LedControlBase()
        led_array = np.zeros(126)
        for i in range(0,126,3):
            led_array[i+1] = 153
            led_array[i+2] = 255
        config.header.stamp = rospy.Time.now()
        config.led_array = led_array.tolist()
        self.pub_base.publish(config)
        self.save_config(led_array)
        rospy.sleep(0.03)
        

    def set_mouth(self,color):
        rospy.loginfo("MOUTH %s" % color.upper())
        color_array = []
        if color == "blue":
            color_array = [0,153,255]
        elif color == "green":
            color_array = [0,255,0]
        elif color == "red":
            color_array = [255,0,0]
        elif color == "yellow":
            color_array = [255,207,0]
        else:
            return
        led_array = np.zeros(12)
        for i in range(4):
            led_array[i*3] = color_array[0]
            led_array[i*3+1] = color_array[1]
            led_array[i*3+2] = color_array[2]
        self.mouth_config.header.stamp = rospy.Time.now()
        self.mouth_config.led_array = led_array.tolist()
        self.pub_mouth.publish(self.mouth_config)
        rospy.sleep(0.05)



    # Yellow 592nm - R 255 G 207 B 0
    def greet(self,TIME):
        self.face("cheerful")
        rospy.sleep(0.4)
        rospy.loginfo("GREETING")
        thread.start_new_thread(self.playsound,("soundfile1.wav",))
        now = rospy.get_time()
        while rospy.get_time() < now + TIME:
            led_array = self.led_config
            for i in range(0,72,3):
                led_array[i] = 255
                led_array[i+1] = 207
                led_array[i+2] = 0
                led_array[-i] = 255
                led_array[-i+1] = 207
                led_array[-i+2] = 0
                self.base_config.header.stamp = rospy.Time.now()
                self.base_config.led_array = led_array.tolist()
                self.pub_base.publish(self.base_config)
                self.save_config(led_array)
                rospy.sleep(0.04)
            self.default_leds()
        self.default_face()


    # Yellow 592nm - R 255 G 207 B 0
    def open_door(self,TIME):
        rospy.loginfo("OPEN DOOR")
        config = LedControlBase()
        thread.start_new_thread(self.playsound,("soundfile5.wav",))
        now = rospy.get_time()
        while rospy.get_time() < now + TIME:
            led_array = self.led_config
            for i in range(14,-1,-1):    
                led_array[i*3] = 255
                led_array[i*3+1] = 207
                led_array[i*3+2] = 0
                config.header.stamp = rospy.Time.now()
                config.led_array = led_array.tolist()
                self.pub_base.publish(config)
                rospy.sleep(0.07)
            self.default_leds()
        self.default_face()


    # Yellow 592nm - R 255 G 207 B 0
    def close_door(self,TIME):
        rospy.loginfo("CLOSE DOOR")
        config = LedControlBase()
        thread.start_new_thread(self.playsound,("soundfile6.wav",))
        now = rospy.get_time()
        while rospy.get_time() < now + TIME:
            led_array = self.led_config
            for i in range(16):
                led_array[i*3] = 255
                led_array[i*3+1] = 207
                led_array[i*3+2] = 0
                config.header.stamp = rospy.Time.now()
                config.led_array = led_array.tolist()
                self.pub_base.publish(config)
                rospy.sleep(0.07)
            self.default_leds()
        self.default_face()
            

    # Green 560nm - R 182 G 255 B 0
    def point(self,direction,TIME):
        self.face("look_%s" % direction)
        rospy.sleep(0.4)
        rospy.loginfo("LOOK %s" % direction.upper())
        now = rospy.get_time()
        while rospy.get_time() < now + TIME:
            led_array = self.led_config
            if direction == "left":
                for i in range(-3,-10,-1):
                    led_array[i*3] = 182
                    led_array[i*3+1] = 255
                    led_array[i*3+2] = 0
            elif direction == "right":
                for i in range(4,11):
                    led_array[i*3] = 182
                    led_array[i*3+1] = 255
                    led_array[i*3+2] = 0
            self.base_config.header.stamp = rospy.Time.now()
            self.base_config.led_array = led_array.tolist()
            self.pub_base.publish(self.base_config)
            rospy.sleep(0.5)
        self.default_leds()
        self.default_face()


    # Yellow 592nm - R 255 G 207 B 0
    def cheerful(self,TIME):
        self.face("cheerful")
        rospy.sleep(0.4)
        rospy.loginfo("CHEERING")
        thread.start_new_thread(self.playsound,("soundfile3.wav",))
        now = rospy.get_time()
        while rospy.get_time() < now + TIME:
            for i in range(42):
                led_array = np.array(self.led_config, copy=True)
                led_array[i*3] = 255
                led_array[i*3+1] = 207
                led_array[i*3+2] = 0
                led_array[(i-1)*3] = 138
                led_array[(i-1)*3+1] = 110
                led_array[(i-1)*3+2] = 0
                led_array[(i-2)*3] = 94
                led_array[(i-2)*3+1] = 75
                led_array[(i-2)*3+2] = 0
                led_array[(i-3)*3] = 48
                led_array[(i-3)*3+1] = 39
                led_array[(i-3)*3+2] = 0
                led_array[(i-4)*3] = 28
                led_array[(i-4)*3+1] = 23
                led_array[(i-4)*3+2] = 0
                self.base_config.header.stamp = rospy.Time.now()
                self.base_config.led_array = led_array.tolist()
                self.pub_base.publish(self.base_config)
                rospy.sleep(0.02)
        self.default_leds()
        self.default_face()


    def no(self,TIME):
        self.face("no")
        rospy.loginfo("NO")
        rospy.sleep(0.4)
        color_values = [30,60,100,160,220,255,255,255,255,255,255,220,160,100,60,30]
        thread.start_new_thread(self.playsound,("soundfile12.wav",))
        now = rospy.get_time()
        while rospy.get_time() < now + TIME:
            led_array = np.array(self.led_config, copy=True)
            for j in range(len(color_values)):
                for i in range(42):
                    led_array[i*3] = color_values[j]
                    led_array[i*3+1] = 0
                    led_array[i*3+2] = 0
                self.base_config.header.stamp = rospy.Time.now()
                self.base_config.led_array = led_array.tolist()
                self.pub_base.publish(self.base_config)
                rospy.sleep(0.05)
        self.default_leds()
        self.default_face()


    def confused(self,TIME):
        self.face("confused")
        rospy.loginfo("CONFUSED")
        rospy.sleep(0.4)
        color_values = [30,60,100,160,220,255,255,255,255,255,255,220,160,100,60,30]
        thread.start_new_thread(self.playsound,("soundfile8.wav",))
        now = rospy.get_time()
        while rospy.get_time() < now + TIME:
            led_array = np.array(self.led_config, copy=True)
            for j in range(len(color_values)):
                for i in range(42):
                    led_array[i*3] = 0
                    led_array[i*3+1] = int(color_values[j]/1.67)
                    led_array[i*3+2] = color_values[j]
                self.base_config.header.stamp = rospy.Time.now()
                self.base_config.led_array = led_array.tolist()
                self.pub_base.publish(self.base_config)
                rospy.sleep(0.05)
        self.default_leds()
        self.default_face()


    def blink(self,color,TIME):
        rospy.loginfo("BLINK %s" % color.upper())
        color_values = [30,60,100,160,220,255,255,255,255,255,255,220,160,100,60,30]
        now = rospy.get_time()
        while rospy.get_time() < now + TIME:
            led_array = np.array(self.led_config, copy=True)
            for j in range(len(color_values)):
                for i in range(42):
                    if color == "red":
                        led_array[i*3] = color_values[j]
                        led_array[i*3+1] = 0
                        led_array[i*3+2] = 0
                    elif color == "green":
                        led_array[i*3] = 0
                        led_array[i*3+1] = color_values[j]
                        led_array[i*3+2] = 0
                    elif color == "blue":
                        led_array[i*3] = 0
                        led_array[i*3+1] = int(color_values[j]/1.67)
                        led_array[i*3+2] = color_values[j]
                    elif color == "yellow":
                        led_array[i*3] = color_values[j]
                        led_array[i*3+1] = int(color_values[j]/1.23)
                        led_array[i*3+2] = 0
                self.base_config.header.stamp = rospy.Time.now()
                self.base_config.led_array = led_array.tolist()
                self.pub_base.publish(self.base_config)
                rospy.sleep(0.05)
        self.default_leds()


    def look_down(self,TIME):
        self.face("look_down")
        rospy.sleep(0.4)
        rospy.loginfo("LOOK DOWN")
        now = rospy.get_time()
        while rospy.get_time() < now + TIME:
            rospy.sleep(0.03)
        self.default_leds()
        self.default_face()


    def think(self,TIME):
        self.face("think")
        rospy.sleep(0.4)
        rospy.loginfo("THINK")
        now = rospy.get_time()
        while rospy.get_time() < now + TIME:
            rospy.sleep(0.03)
        self.default_leds()
        self.default_face()


    def stop(self):
        self.face("blank")
        self.reset_leds()


    def listener(self):
        while not rospy.is_shutdown():
            if self.callback_flag:
                if self.message == "red" or self.message == "green" or self.message == "blue" or self.message == "yellow":
                    self.blink(self.message,5)
                elif self.message == "greeting":
                    self.greet(7)
                elif self.message == "open" and not self.door_opened:
                    self.open_door(5)
                    self.door_opened = True
                elif self.message == "close" and self.door_opened:
                    self.close_door(5)
                    self.door_opened = False
                elif self.message == "left" or self.message == "right":
                    sq.point(self.message,3)
                elif self.message == "cheerful":
                    self.cheerful(5)
                elif self.message == "think":
                    self.think(5)
                elif self.message == "down":
                    self.look_down(5)
                elif self.message == "no":
                    self.no(5)
                elif self.message == "confused":
                    self.confused(5)
                elif self.message[0:5] == "mouth":
                    self.set_mouth(self.message[6:])
                self.callback_flag = 0
            rospy.sleep(0.05)



################ TESTING FUNCTION ########################
    def play_animation(self):
        config = LedControlBase()
        while not rospy.is_shutdown():
            for i in range(42):
                led_array = np.zeros(126)
                led_array[i*3+2] = 255
                led_array[(i-1)*3+2] = 200
                led_array[(i-2)*3+2] = 150
                led_array[(i-3)*3+2] = 100
                led_array[(i-4)*3+2] = 50
                config.header.stamp = rospy.Time.now()
                config.led_array = led_array.tolist()
                self.pub_base.publish(config)
                rospy.sleep(0.03)
###############################################################


if __name__ == "__main__":

    rospy.init_node("squirrel_idmind_control")
    sq = SquirrelControl()
    rospy.on_shutdown(sq.stop)
    sq.default_face()
    sq.default_leds()
    sq.listener()