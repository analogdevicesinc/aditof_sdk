# Utilities for Dragonboard

This directory contains different utilities may be needed on the Dragonbard.

#### List of Utilities

| Name | Description |
| --------- | -------------- |
| config_pipe.sh | Configures the video data pipeline coming from the camera |
| config_pipe.service | A 'systemd' startup script that executes config-pipe.sh when linaro boots |
| 53-aditofsdkdragonboard.rules | Allows read/write access to the eeprom and the temperature sensor |

To configure config-pipe.sh to be executed automatically on boot:

1. copy the service file to systemd
```
sudo cp config_pipe.service /etc/systemd/system/
```
2. run:
```
sudo systemctl enable config_pipe
```

If you don't want to wait until you reboot the board so the changes take effect, you can start the service right away:
1. run:
```
sudo systemctl start config_pipe
```
