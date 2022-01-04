# First-frame-network python example

##Overview

This is a python binding of the C++ first-frame-network example. It uses client-server communication for retrieving the first frame captured by a remote ToF camera.

The main difference from first-frame example is the use of the function "system.getCameraListAtIp(cameras, ip)" which needs an ip address in order to connect with the camera.

This ip address is passed as an argument for the script. For example, if the address of the target camera is `10.42.0.161`, then the script must be called like this:
`python3 first_frame_network.py 10.42.0.161`.
