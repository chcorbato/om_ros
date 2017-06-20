#!/usr/bin/env python

'''
@author: Carlos Hernandez
@organization: ASLab
@summary: module that realises perception up to the sensory processing, on rosnodes
@status: second version - working
'''

#import sys
#import os
#import xmlrpclib

import roslib; roslib.load_manifest('OMRosDrivers_py')
import rospy
import rosnode
import roslib.scriptutil as scriptutil

from actuation import Actuator


#####################################################################
def main():
        
    rospy.init_node('meta_actuator')
    rospy.loginfo("actuator node started...")
  
    #--For acting --------------------------------------------------------------
    actuator = Actuator()
    
    # working loop --------------------------------------------------------------------
    while not rospy.is_shutdown():
        rospy.sleep(2)
        
    # termination -----------------------------------------------------------------
    rospy.loginfo("...actuator ended")
    
    
#####################################################################
    
if __name__ == '__main__':
    main()