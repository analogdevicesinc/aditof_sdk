# test-fxtof1 production line testing application

## Overview

camera-test is an application used by the factory to test the quality of the camera, 
calibration, as well as the eeprom content.

In order for the latest frame (both IR and DEPTH image) to be saved as well as the measured
distance, the cmake command must contain -DDATA_HANDLING=1
