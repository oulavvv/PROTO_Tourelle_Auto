#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64, Bool, Float32
from dynamixel_msgs.msg import JointState

class turnServo():

    def __init__(self):

        rospy.init_node('turnServo', anonymous=True)
        self.position = Float64()
        
        self.posFlag = False
        self.done = True
        self.stop = False #bloque si tof personne trop proche
        self.modeManuel = True

        self.pub_pos = rospy.Publisher('/tilt_controller_trigger/command', Float64 , queue_size=1)
        self.sub_posState = rospy.Subscriber("/tilt_controller_trigger/state", JointState, self.callback)
        self.sub_shoot_auto = rospy.Subscriber("/filter_person", Bool, self.callback_shoot_auto)
        self.sub_shoot_manuel = rospy.Subscriber("/positionTir", Bool, self.callback_shoot_manuel)
        self.sub_tof = rospy.Subscriber("/tof_dist", Float32, self.callback_tof)
        self.sub_switchMode = rospy.Subscriber("/statusSwitch", Float64, self.callback_switchMode)
        

        time.sleep(3)
        self.pubPositionInit()


    def callback_switchMode(self, data_switch):
        if data_switch.data == 0:
            print("manuel")
            self.modeManuel = True
        elif data_switch.data == 1:
            print("auto")
            self.modeManuel = False


    def callback_tof(self, data):
        if data.data < 400 :
            self.stop = True
        else :
            self.stop = False


    def callback_shoot_auto(self,data):
        if self.modeManuel == False :
            if data.data == True and self.done == True:
                self.done = False
                self.position.data = -1.02
                self.pubPosition()


    def callback_shoot_manuel(self,data):
        if self.modeManuel == True :
            if data.data == True and self.done == True:
                self.done = False
                self.position.data = -1.02
                self.pubPosition()


    def callback(self,data_state):
        print(abs(data_state.goal_pos - data_state.current_pos))
        if abs(data_state.goal_pos - data_state.current_pos) < 0.08:
            self.position.data = -1.99
            self.pubPosition()
            #time.sleep(3)
            self.done = True
    

    def pubPositionInit(self):
        self.pub_pos.publish(self.position)
        self.position.data = -1.99
        self.pubPosition()


    def pubPosition(self):
        if self.stop == False :
            self.pub_pos.publish(self.position)


if __name__ == '__main__':
    pos = turnServo()
    # spin() simply keeps python from exiting until this node is stopped
    # pos.pubPosition()
    rospy.spin()