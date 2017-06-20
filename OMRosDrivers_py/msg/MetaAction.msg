# This message is to send standard unitary actions to the OM actuator
Header header #for timestamp

int16 actionID 	#to identify each unitary action (better use seq in header for that)
string actionName # KILL and LAUNCH for the moment
string nodeName   # if action is KILL
string pckg		# if action is LAUNCH, the package that contains the launchfile
string launchfile	# if action is LAUNCH, name of the launchfile