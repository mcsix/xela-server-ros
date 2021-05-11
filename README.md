# XELA Sensor Server ROS module package
## Disclaimer
This package is released As Is and under GNU GPL v3 license.
## Purpose
This package is used to convert XELA Server packets into ROS Service messages.
## Warnings
ROS has limits on how many messages it is able to send per second, which can be a lot less than what the server app does. Therefore this app has to dump excess messages by overwriting the buffer.
There are several residual parts from initial closed source module with confidential parts stripped out.
## Requirements
Python 2.7 (not recommended) or Python 3.4+
`websocket-client` package
## Use
### Installation
Copy the `xela_server` directory into your catkin workspace `src` directory and build as usual.
### Run
Either run the launch file `roslaunch xela_server service.launch` or activate everything separately `roscore`, `xela_server` and `rosrun xela_server xela_service`
> __Note__ For the launch file, the xela_server app must be in the `path` variable or you would need to replace the `xela_server` app inside the `scripts` directory with the one from the main app suite
### Services
| __Service call__ | __Explanation__ |__Example output__ |
| --- | --- | --- |
| `rosservice call /xServXY 1 2` | Get X and Y for Taxel 2 on Sensor 1 | values: [32572, 31596] |
| `rosservice call /xServXYZ 1 2` | Get X, Y and Z for Taxel 2 on Sensor 1 | values: [32572, 31596, 35901] |
| `rosservice call /xServX 1 2` | Get X for Taxel 2 on Sensor 1 | value: 32572 |
| `rosservice call /xServY 1 2` | Get Y for Taxel 2 on Sensor 1 | value: 31596 |
| `rosservice call /xServZ 1 2` | Get Z for Taxel 2 on Sensor 1 | value: 35901 |
| `rosservice call /xServStream 1` | Get Full data from Sensor 1 | xyz: [1: [32457, 32553, 32057], 2: [32572, 31596, 35901]… ] |

### Topics
| __Topic name__ |  __Topic message type__ | __Description__ |
| --- | --- | --- |
| `/xServTopic` | `xela_server/XStream` | Get all sensors and taxels as a constant stream |

### ROS Launch arguments
| __Argument__ | __Description__ |
| --- | --- |
| noserver:=1 | Will not run xela_server (requires external launch of the app on same or different computer)|
| port:=5000 | Define xela_server port, default is 5000 |
| ip:=127.0.0.1 | Define xela_server IP, default is 127.0.0.1 |
| rbport:=9090 | Define ros_bridge port, default is 9090 |
| rbip:=localhost | Define ros_bridge IP, default is localhost |
| file:=/etc/xela/xServ.ini | Define configuration file for xela_server, default is /etc/xela/xServ.ini |
| d:=0 | Define debug level, default 0, max 3. Prints info based on service requests made via ROS <br>1 - Request type only <br>2 - Request type and sensor/taxel info <br>3 - Request type, sensor/taxel info and returned values |

### Example code for ROS
```python
#!/usr/bin/env python

import rospy from xela_sensors.srv
import XelaSensorXYZ
import sys

rospy.init_node('use_service') 

#wait the service to be advertised, otherwise the service use will fail 
rospy.wait_for_service('xServXYZ') 

#setup a local proxy for the service (we will ask for X,Y and Z data) 
srv=rospy.ServiceProxy('xServXYZ', XelaSensorXYZ) 

#use the service and send it a value. 
#In this case, I am sending sensor: 1 and taxel: 3 
service_example=srv(1, 3) 

#print the result from the service 
print(service_example) 

#close the app 
sys.exit(0)
```