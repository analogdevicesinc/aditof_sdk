# Tof daemon

The daemon is written to be run on the Dragonboard.

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
