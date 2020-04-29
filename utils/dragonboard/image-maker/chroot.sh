#!/bin/bash

qemu-aarch64-static /bin/bash

su - linaro

sudo apt-get update -yy -qq
sudo apt-get upgrade -yy -qq
sudo apt-get install v4l-utils -yy -qq
sudo apt-get install cmake -yy -qq
sudo apt-get install wget -yy -qq
sudo apt-get install xz-utils -qq
sudo apt-get install -yy -qq python3-pandas python3-natsort python3-matplotlib python3-tk python3-scipy python3-seaborn python3-serial
sudo apt-get install -yy -qq libssl-dev python-dev python3-dev build-essential libgtk2.0-dev pkg-config

cd /home/linaro/workspace/github

wget http://swdownloads.analog.com/cse/aditof/deps-dragonboard.tar.xz
tar -xf deps-dragonboard.tar.xz

cp -r /home/linaro/workspace/github/deps/opencv-3.4.1/* /usr/local/
cp -r /home/linaro/workspace/github/deps/glog/* /usr/local/
cp -r /home/linaro/workspace/github/deps/protobuf/* /usr/local/
cp -r /home/linaro/workspace/github/deps/websockets/* /usr/local/

cp -r /home/linaro/workspace/github/deps/py36tofcalib/lib/python3.5/site-packages/modbus_tk /usr/lib/python3/dist-packages/
cp -r /home/linaro/workspace/github/deps/py36tofcalib/lib/python3.5/site-packages/modbus_tk-1.0.0.dist-info /usr/lib/python3/dist-packages/

cp -r /home/linaro/workspace/github/deps/py36tofcalib/lib/python3.5/site-packages/click /usr/lib/python3/dist-packages/
cp -r /home/linaro/workspace/github/deps/py36tofcalib/lib/python3.5/site-packages/Click-7.0.dist-info /usr/lib/python3/dist-packages/

mkdir -p /home/linaro/.config/

cp -r /home/linaro/workspace/github/deps/chromium /home/linaro/.config/

sudo rm -rf /home/linaro/workspace/github/deps
sudo rm -rf /home/linaro/workspace/github/deps-dragonboard.tar.xz

cd aditof_sdk

mkdir -p build
cd build
cmake .. -DDRAGONBOARD=1 -DWITH_PYTHON=on
make -j8
sudo make install-udev-rules

cd /home/linaro/workspace/github/aditof_sdk/apps/daemon
sudo cp tof-programming.service /etc/systemd/system/
mkdir -p build
cd build
cmake ..
make -j8
sudo systemctl enable tof-programming

cd /home/linaro/workspace/github/aditof_sdk/utils/dragonboard
sudo cp config_pipe.service /etc/systemd/system/
sudo systemctl enable config_pipe

mkdir -p /home/linaro/Desktop
cd /home/linaro/Desktop
touch aditof-demo.sh
echo '#!/bin/bash' >> aditof-demo.sh
echo 'cd /home/linaro/workspace/github/aditof_sdk/build/examples/aditof-demo' >> aditof-demo.sh
echo './aditof-demo' >> aditof-demo.sh
sudo chmod +x aditof-demo.sh

sudo cp /home/linaro/info.txt /home/linaro/Desktop
sudo rm /home/linaro/info.txt

# fix
sudo chown -R linaro:linaro /home/linaro/workspace
sudo chown -R linaro:linaro /home/linaro/.config
sudo chown -R linaro:linaro /home/linaro/Desktop
