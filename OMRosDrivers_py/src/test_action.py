#!/usr/bin/env python

'''
Created on Nov 10, 2011

@author: chcorbato
@summary: ros node that publishes MetaAction messages from console input
          for testing purposes
'''

import roslib; roslib.load_manifest('OMRosDrivers_py')
import rospy

import sys
import getopt

from OMRosDrivers_py.msg import MetaAction, MetaActionResult

def result_CB(msg):
    print "action-" + str(msg.actionID) + " --> " + msg.result
    print "\t log:\t" + msg.log 
    
def main():
    rospy.init_node('meta_action_tester')   

    action_pub = rospy.Publisher('/meta_action', MetaAction)
    
    result_sub = rospy.Subscriber('/meta_action_result', MetaActionResult, result_CB)
    
    n_actions = 0

    print "enter:\n k to kill a node\n l to launch a launchfile\n q to quit"
         
    #------------------------------------------------------        
    # working loop --------------------------------------------------------------------
    while not rospy.is_shutdown():
        #rospy.sleep(0.01)
        if raw_input() == 'k':
            node = raw_input("write the full name of the node to kill:\n")
            action = MetaAction()
            action.actionID = n_actions
            n_actions = n_actions + 1
            action.actionName = 'KILL'
            action.nodeName = node
            action_pub.publish(action)
            print "command sent to kill node: " + node
            
        if raw_input() == 'l':
            pkg = raw_input("write the name of the package:\n")
            launchfile = raw_input("write the name of the launchfile:\n")
            action = MetaAction()
            action.actionID = n_actions
            n_actions = n_actions + 1
            action.actionName = 'LAUNCH'
            action.pckg = pkg
            action.launchfile = launchfile
            action_pub.publish(action)
            print "command sent to launch:  " + pkg + "  " + launchfile
            
        if raw_input() == 'q':
            rospy.loginfo("node ended")
            return

    # termination -----------------------------------------------------------------
    rospy.loginfo("meta_action tester ended")       
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass


