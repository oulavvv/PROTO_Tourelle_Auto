#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64,Bool
from dynamixel_msgs.msg import JointState

class turnServo():

    def __init__(self):

        rospy.init_node('turnServo', anonymous=True)
        self.position = Float64()
        self.position.data = 0
        self.init_auto = False
        self.posFlag = False
        self.modeManuel = True
        self.pub_pos = rospy.Publisher('/tilt_controller_base/command', Float64 , queue_size=1)
        self.sub_posState = rospy.Subscriber("/tilt_controller_base/state", JointState, self.callback)
        self.sub_posAPP = rospy.Subscriber("/positionApp",Float64,self.callbackPos)
        self.sub_switchState = rospy.Subscriber("/statusSwitch",Float64,self.callbackSwitch)
        time.sleep(3)
        self.pubPositionInit()

    def callback(self,data_state):
        #print("mode choisi", self.modeManuel)
        #print(self.position)
        if self.modeManuel == True:
            
            self.pubPosition()
            self.init_auto = False

        else:
            if self.init_auto == False:
                self.pubPositionInitAuto()
                self.init_auto = True
            if abs(data_state.goal_pos - data_state.current_pos) < 0.1:
                self.position.data = - self.position.data
                time.sleep(6)
                self.pubPosition()

    def callbackPos(self,data_pos):
        self.position.data = data_pos.data

    def callbackSwitch(self,data_switch):
        print('in switch')
        if data_switch.data == 0:
            print("manuel")
            self.modeManuel = True
        elif data_switch.data == 1:
            print("auto")
            self.modeManuel = False

    def pubPositionInit(self):
        self.pub_pos.publish(0)

    def pubPositionInitAuto(self):

        self.position.data = 3
        self.pubPosition()

    def pubPosition(self):

        self.pub_pos.publish(self.position)


def Main():

    pos = turnServo()
    # spin() simply keeps python from exiting until this node is stopped
    # pos.pubPosition()
    rospy.spin()

if __name__ == '__main__':
    Main()