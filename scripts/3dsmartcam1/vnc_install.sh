#!/bin/bash

sudo apt-get update -y
sudo apt-get install -y x11vnc net-tools
echo '/usr/sbin/lightdm' | sudo tee /etc/X11/default-display-manager > /dev/null 
sudo dpkg-reconfigure -fnoninteractive lightdm
x11vnc -storepasswd

#create systemd service for starting the VNC server and enable it
sudo bash -c 'cat > /etc/systemd/system/x11vnc.service' << EOF
[Unit]
Description="x11vnc"
Requires=display-manager.service
After=display-manager.service

[Service]
ExecStart=/usr/bin/x11vnc -loop -nopw -xkb -repeat -noxrecord -noxfixes -noxdamage -forever -rfbport 5900 -display :0 -auth guess
ExecStop=/usr/bin/killall x11vnc
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl enable x11vnc.service

#create virtual display to set default resolution to 1920x1080
sudo bash -c 'cat > /usr/share/X11/xorg.conf.d/10-monitor.conf' << EOF
Section "Screen"
  Identifier "Screen0"
  Device "Tegra0" # e.g. Radeon, Intel, nvidia
  Monitor "DP"
  DefaultDepth 24
  SubSection "Display"
    Depth 24
    Virtual 1920 1080 # 1920 + 1680 (3600), 1080 + 1050 (2130)
  EndSubSection
EndSection
EOF
