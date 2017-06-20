#!/usr/bin/env python

'''
Created on Nov 10, 2011

@author: chcorbato
@summary: ros node that publishes MetaAction messages from console input
          for testing purposes
'''

import roslib; roslib.load_manifest('OMRosDrivers_py')
import rospy

import sys, struct
import getopt

# for the sensing messages -----------------------------------
from OMRosDrivers_py.msg import PerceptionFlow, RosSingularity, ROSTopic, ROSParameter, ComponentReferent, ComponentReferents, OMTypes 

OMTYPES = OMTypes()
known_types = [OMTYPES.INTEGER, OMTYPES.DOUBLE, OMTYPES.STRING]

readings = []

#####################################################################
def sensing_CB(msg):
    global readings
    readings = msg.singularities
        
#####################################################################
def main():
    rospy.init_node('meta_sensor_tester')   

    referents_pub = rospy.Publisher("/meta_referents", ComponentReferents)
    
    sensor_sub = rospy.Subscriber('/meta_singularities', PerceptionFlow, sensing_CB)

    print "enter:\n n to add a new referent \n s to get latest sensing \n q to quit"
         
    #------------------------------------------------------        
    # working loop --------------------------------------------------------------------
    while not rospy.is_shutdown():
        a = 'a'
        a = raw_input()
        if a == 'n':
            sref = raw_input("\n -> write the full name of the referent to add:\n")
            if len( sref.strip() ) > 0:
                t = raw_input("\t write OBSERVE to observe it, IGNORE to ignore it:\n")
                if t <> 'OBSERVE' and t<>'IGNORE' :
                    print '\n \t wrong input \n'
                    continue
                referent = ComponentReferent(sref, None, t)
                referents = ComponentReferents(None, [referent])
                referents_pub.publish(referents)
                print "new referent to " + t + " provided: " + referent.name
            print "\n enter:\n n to add a new referent \n s to get latest sensing \n q to quit"

        
        if a == 's':
            print "sensing received: -----------------------"
            for x in readings:
                print "-----------"
                print "Component:  " + x.node_name
                print 'status: ' + str(x.status)
                print 'parameters:'
                for p in x.params:
                    if p.type not in known_types:
                        print '\t' + p.name + '\t UNSUPPORTED TYPE'
                    else:
                        value = struct.unpack(p.type, p.value)
                        print '\t' + p.name + '\t' + repr(value[0])
                print 'connectors:'
                for t in x.subs:
                    print '\t' + t.name + ' (IN)'
                for t in x.pubs:
                    print '\t' + t.name + ' (OUT)'
                print 'log:'
                for s in x.errors :
                    print '\t' + s   
            print "-----------------------------------------"
            print "\n"+"enter:\n n to add a new referent \n s to get latest sensing \n q to quit"

        if a == 'q':
            rospy.loginfo("node ended")
            return
        
    # termination -----------------------------------------------------------------
    rospy.loginfo("meta_action tester ended")       
    

if __name__ == '__main__':
    main()
