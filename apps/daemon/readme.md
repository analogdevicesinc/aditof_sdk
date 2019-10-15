# Tof daemon

For the target to be discoverable on the USB bus of a PC host, a certain program (uvc-gadget) needs to be run on the target. This daemon allows for the uvc-gadget to be started with a push of a button on the Time of Flight board.
This is done by monitoring the activity of a user hardware button on the Time of Flight mezzanine board through a GPIO driver. When the button gets pushed the daemon starts the uvc-gadget application and turns on a LED on the mezzanine. The uvc-gadget is terminated and the LED is turned off on the second push of the button.
The daemon is written to be run on the Dragonboard only.

To make the daemon automatically start on boot:

1. copy the service file to systemd
```
sudo cp tof-programming.service /etc/systemd/system/
```
2. run:
```
sudo systemctl enable tof-programming
```

If you don't want to wait until you reboot the board so the changes take effect, you can start the service right away:
1. run:
```
sudo systemctl start tof-programming
```
