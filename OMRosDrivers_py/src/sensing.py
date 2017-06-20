'''
Created on Sep 29, 2011

@author: chcorbato
'''

import roslib; roslib.load_manifest('OMRosDrivers_py')
import rospy
import rosnode
import rosparam
import roslib.scriptutil as scriptutil

from rosgraph_msgs.msg import Log

# for the sensing perception ------------------------------
from OMRosDrivers_py.msg import PerceptionFlow
from OMRosDrivers_py.msg import RosSingularity
from OMRosDrivers_py.msg import ComponentReferents
from OMRosDrivers_py.msg import ROSTopic
from OMRosDrivers_py.msg import ROSParameter, OMTypes
#----------------------------------------------------------

import socket, struct

#####################################################################

known_types = [int, float, str]

OMTYPES = OMTypes()

class OMRosSensor():
    
    
    ID = '/rosnode'
    
    singularities = []
    refs2ignore = []
    refs2observe = []
    logBuffer = []

    def __init__(self):   
        # subscribers to sensing topics
        self.subs_rosout = rospy.Subscriber("/rosout", Log, self.error_sensing_CB)
        self.subs_rosoutagg = rospy.Subscriber("/rosout_agg", Log, self.error_sensing_CB)
        
        # subscriber for refs2ignore and singularities
        self.subs_referents = rospy.Subscriber("/meta_referents", ComponentReferents, self.updateReferents_CB)
      
        # publisher for singularities
        self.pub_singularities = rospy.Publisher('/meta_singularities', PerceptionFlow)
        
        self.refs2ignore = ['/rosout', '/meta_sensor_node', '/meta_actuator_node', '/meta_controller']
               

    
    #####################################################################   
    def filterOut(self, sensing, criteria):
    # @summary: function that removes from the sensing array the elements that are in criteria
        for x in criteria:
            if x in sensing:
                sensing.remove(x)
        return sensing


    #####################################################################   
    def sense(self):
    # @summary: basic cycle of the OMRosSensor
        
        # sense the components      
        sensed_nodes = rosnode.get_node_names()
        
        #for the moment we do not monitor this node itself
        filtered_nodes = self.filterOut(sensed_nodes, self.refs2ignore);
        #print filtered_nodes
        #print '\n'
        # generate singularities
        self.singularities = [self.get_node_info(node_name=x) for x in filtered_nodes]
        
        # actively try to sense the referents to observe
        for x in self.refs2observe :
            if x not in filtered_nodes : # if not sensed it is in 
                sing = RosSingularity(x, 0, None, None, None, [])
                self.singularities.append(sing)
    
        perceptual_flow = PerceptionFlow(None, self.singularities)
        #print self.singularities
        #print "\n ------------------- \n"
        # publish singularities
        self.pub_singularities.publish(perceptual_flow)
        self.logBuffer = []
        
    
        # print '---- cycle ---------------------------------------------------\n \n'


    #####################################################################
    
    def get_node_info(self, node_name):
        def topic_type(t, pub_topics):
        #@summary: auxiliary function to obtain the type of a topic
            matches = [t_type for t_name, t_type in pub_topics if t_name == t]
            if matches:
                return matches[0]
            return 'unknown'
        
        master = scriptutil.get_master() # obtain the ros master
            
        # @todo: would be interesting to rosnode_ping if the node is not accessible from master send node in error
        # but the method hangs, don't know why
        
        # go through the master system state first
        try:
            pub_topics = master.getPublishedTopics(self.ID, '/')
            pub_topics = pub_topics[2]
            state = master.getSystemState(self.ID)
            state = state[2]
            
        except socket.error:
            raise ROSNodeIOException("Unable to communicate with master!")
        
        # gather info about the node parameters
        # use: rosparam.list_params(ns) and rosparam.get_param(param), ns= name space of the node
        param_names = rosparam.list_params(node_name)
        params = []
        for p in param_names:
            parameter = ROSParameter()
            parameter.name = p.replace(node_name+'/', "")
            v = rosparam.get_param(p)
            if type(v) in known_types :
                if type(v) is int :
                    parameter.type = OMTYPES.INTEGER
                elif type(v) is str :
                    parameter.type = OMTYPES.STRING
                elif type(v) is float :
                    parameter.type = OMTYPES.DOUBLE
                parameter.value = struct.pack(parameter.type, v)
            else :
                parameter.value = []
            params.append(parameter)
            
        # srvs = [t for t, l in state[2] if node_name in l] #eventually for services   
        
        # lists of pub and subs topics names
        pubs = [t for t, l in state[0] if node_name in l]
        subs = [t for t, l in state[1] if node_name in l]        
        # create double list with topic names and types
        published = None
        subscribed = None
        if len(pubs) <> 0 :
            published = [ROSTopic(p, topic_type(p, pub_topics)) for p in pubs]
        if len(subs) <> 0 :   
            subscribed = [ROSTopic(p, topic_type(p, pub_topics)) for p in subs]
            
        # add log information 
        # @todo: maybe also filter with errors from refs2ignore
        errors = []    
        for l in self.logBuffer:
            if l.name == node_name:
                level = ''
                if l.level == 8:
                    level = ' ERROR'
                entry = l.msg + level
                errors.append( entry )
        
        return RosSingularity(node_name, 1, subscribed, published, params, errors)

    
    #####################################################################
    # CallBacks
    #####################################################################
    def error_sensing_CB(self, data):
        self.logBuffer.append(data)
        
                
    #####################################################################
    def updateReferents_CB(self, s):     
        rospy.loginfo("Updated referents: ")
        self.refs2observe = self.refs2observe + [ x.name for x in s.status if x.type=='OBSERVE' and (x.name not in self.refs2observe) ]
        self.refs2ignore = self.refs2ignore + [ x.name for x in s.status if x.type=='IGNORE' and (x.name not in self.refs2ignore) ]  
        for x in self.refs2observe :
            if x in self.refs2ignore :
                self.refs2ignore.remove(x)
        print "actively observing: "     
        print self.refs2observe
        print "ignoring: "
        print self.refs2ignore
        
