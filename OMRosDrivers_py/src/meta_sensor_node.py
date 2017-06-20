#!/usr/bin/env python

'''
@author: Carlos Hernandez
@organization: ASLab
@summary: module that realises perception up to the sensory processing, on rosnodes
@status: second version - basic functionality but working
'''

#import sys
#import os
#import xmlrpclib

import roslib; roslib.load_manifest('OMRosDrivers_py')
import rospy
import rosnode
import roslib.scriptutil as scriptutil 


# for the sensing perception ------------------------------
# sensing is performed in a cycle with continuous sending of meta_sensing info
from sensing import OMRosSensor
#----------------------------------------------------------


#####################################################################
def main():
        
    rospy.init_node('meta_sensor')
    rospy.loginfo("sensing node started...")
  
    #--For sensing --------------------------------------------------------
    sensor = OMRosSensor()
    
    # working loop --------------------------------------------------------
    while not rospy.is_shutdown():
        sensor.sense()
        rospy.sleep(2)
        
    # termination ---------------------------------------------------------
    rospy.loginfo("...meta_sensor ended")
    
    
#####################################################################
    
if __name__ == '__main__':
    main()