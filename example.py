#!/usr/bin/env python
 
import rospy
from xela_server.srv import XelaSensorXYZ
import sys
 
rospy.init_node('use_service')
 
# wait the service to be advertised, otherwise the service use will fail
rospy.wait_for_service('xServXYZ')
 
# setup a local proxy for the service (we will ask for X,Y and Z data)
srv=rospy.ServiceProxy('xServXYZ', XelaSensorXYZ)
 
# use the service and send it a value. 
# In this case, I am sending sensor: 1 and taxel: 3
service_example=srv(1, 3)
 
# print the result from the service
print(service_example)

# close the app
sys.exit(0)
