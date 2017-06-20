# This message holds the status of an individual component of the robot.
# 

# Possible levels of operations
byte OK=1
byte ERROR=0

string node_name # a description of the test/component reporting
byte status # status of the ros node regarding master connectivity
ROSTopic[] subs
ROSTopic[] pubs
ROSParameter[] params # an array of values associated with the status
string[] errors # latest history of relevant messages sent to rosout from the node
