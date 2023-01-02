#!/usr/bin/env python

import time
import rospy
from process.msg import BoundingBoxes, ObjectCount
from std_msgs.msg import Bool

class Filter:

    def __init__(self):
        self.isHere = False
        rospy.init_node('filter_person', anonymous=True)   
        self.pub_filter = rospy.Publisher('/filter_person', Bool, queue_size = 1)
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.callback_boxes)
        
        self.publisher()


    def callback_boxes(self, data):  
        #ficher config uniquement avec person donc callback seulement si une personne est detectee
        
        self.x_center = (data.bounding_boxes[0].xmin + data.bounding_boxes[0].xmax)/2
        self.y_center = (data.bounding_boxes[0].ymin + data.bounding_boxes[0].ymax)/2
        self.center = [self.x_center, self.y_center]
        #print(self.center)
        self.isHere = True
         

    def callback_count(self, data): 
        #print("count : "+str(data.count))
        self.count = data.count
        self.callback_boxes(self.count)
        

    def publisher(self):
        while not rospy.is_shutdown():
            if self.isHere == True :
                time.sleep(5)
                self.pub_filter.publish(self.isHere)
                self.isHere = False


if __name__ == '__main__':
    o = Filter()
    rospy.spin()
