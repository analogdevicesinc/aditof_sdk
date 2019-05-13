# Tof daemon

The daemon is written to be run on the Dragonboard.

To make the daemon automatically start on boot:

1. copy tof-programming.service to /etc/systemd/system/
2. run:
systemctl enable tof-programming

If you don't want to wait until reboot, the service can be started right away:
Run:
systemctl start tof-programming

