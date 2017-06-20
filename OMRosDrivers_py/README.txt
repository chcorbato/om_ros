Author: Carlos Hernandez @ www.aslab.org
creation date: May 18 2012

last updated: February 28 2013
		Carlos Hernandez
		comment: increased tout param of runCommand in ch_syscall to capture the log when fails
		TODO: solve possible problems with this parameter 

USAGE
=====
To add a new field to any of the user-defined ros messages (e.g. MetaActionResult.msg) you have to 
modify the .msg file and then, before running rosmake for the package, clean up the files corresponding
to the package (not only those of that message) in ~/.ros/rosjava, because it seems that there is a bug in the rosjava compilation
that those files are not overwritten if they already exist.
Then, the rosjava package must be re-compiled ($ rosmake rosjava)

INFO
====
This project contains the ROS nodes for reflection: monitoring and reconfiguring nodes in a ROS system


DOCUMENTATION - needs to be updated
=============
More documentation about this project and other projects related to the OM Architecture can be found at:
svn+ssh://chcorbato@software.aslab.upm.es/home/svnroot/people/chcorbato/models/OM_Models/docs
