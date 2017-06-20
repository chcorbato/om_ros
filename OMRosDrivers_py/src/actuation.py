'''
Created on Sep 29, 2011

@author: chcorbato
'''

import roslib; roslib.load_manifest('OMRosDrivers_py')
import rospy
import rosnode
import roslib.scriptutil as scriptutil

from OMRosDrivers_py.msg import MetaAction, MetaActionResult

from ch_syscall import runCommand

from rosgraph_msgs.msg import Log

#############################################################################

class Actuator():

    def __init__(self):  
        # subscriber for action
        self.action_subs = rospy.Subscriber("/meta_action", MetaAction, self.action_CB)
        # publisher for action result
        self.action_result_pub = rospy.Publisher('/meta_action_result', MetaActionResult)
    

    #############################################################################
    # action CB handling
    def action_CB(self, msg):
        action_str = "\n action " + str(msg.actionID) + " " + msg.actionName + " requested for:"
        result = MetaActionResult()
        result.header.stamp = rospy.get_rostime()
        result.actionID = msg.actionID
        # @todo: add waits for action completion and publishing of action result
        if msg.actionName == 'KILL':
            action_str = action_str + "\n \t node: " + msg.nodeName
            if self.kill_node(msg.nodeName):
                result.result = "SUCCEEDED"
            else:
                result.result = "FAILED"
                
        if msg.actionName == 'LAUNCH':
            action_str = action_str + "\n \t launchfile: " + msg.pckg + "/" + msg.launchfile
            r, result.log = self.launch(msg.pckg, msg.launchfile)            
            if r:
                result.result = "SUCCEEDED"
            else:
                result.result = "FAILED"
            
        feedback = "\n \t result: " + result.result + "\n \t output: " + result.log
        rospy.loginfo( action_str + feedback )
        self.action_result_pub.publish(result)
             
    
    
    #############################################################################
    # methods for handling the KILL action_str
    
    def kill_node(self, x):
        '''
        @param x: a single string for a node name 
        @return: a pair - result (1 if successful, 0 otherwise) and an empty string for the log
       '''
        a=[x]
        (success, failures) = rosnode.kill_nodes(a)
        if success:
            result = 1
            log = ""
        else:
            result = 0
            log = ""
        return (result, log)
     
    def kill_node_by_command(self, x):
        # @todo: deprecated, eliminate, kill_node replaces it
        cmd = 'rosnode kill ' + x + ' &' 
        print cmd
        runCmd(cmd)
   
        
    #############################################################################
    # methods for handling the LAUNCH action
        
    def launch(self, rospack, launchfile):
        '''
        @param rospack: the package that contains the launchfile
        @param rospack: the name of the launchfile
        @return: a pair: result (1 success, 0 fail) and stdout of the call
        '''
        cmd = "roslaunch " + rospack + " " + launchfile + ' &'
        result, log = runCommand(cmd) # obtain command output
        return (result, log)

    
