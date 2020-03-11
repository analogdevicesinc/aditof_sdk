# Utilities for Raspberry Pi

This directory contains different utilities that may be needed on the Raspberry Pi.

#### List of Utilities

| Name | Description |
| --------- | -------------- |
| tof-server.service | A 'systemd' startup script that starts the Time of Flight server when raspbian boots |
| 53-aditofsdkraspberrypi.rules | Allows read/write access to the eeprom and the temperature sensor |

To enable the service:

1. copy the service file to systemd
```
sudo cp tof-server.service /etc/systemd/system/
```
2. run:
```
sudo systemctl enable tof-server
```

If you don't want to wait until you reboot the board so the changes take effect, you can start the service right away:
1. run:
```
sudo systemctl start tof-server
```
