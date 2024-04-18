# Cuda binding for Smart Camera 3D
 Eqipment: 
 1. Analog Devices Smart Camera (Jetson Nano and FXTOF1)
 2. SSH connection to camera

## Using CUDA cores to optimize frame corrections

1. Clone the repository
2. Create bild directory and use the following cmake command for ```3D Smart Camera```:
```console
cmake -DUSE_3D_SMART=1 -DCUDA_ON_TARGET=1 -DWITH_EXAMPLES=1 -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..

```
or the following command for ```Jetson Nano``` and ```FXTOF1``` setup:
```console
cmake -DJETSON=1 -DUSE_FXTOF1=1 -DCUDA_ON_TARGET=1 -DWITH_EXAMPLES=1 -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..

```
3. Make the project with:
```console
make -j8
```
In order to disable the CUDA cores, set the ```-DCUDA_ON_TARGET``` flag to false.

4. Install Nvidia-GPU driver api and CUDA:
```console
    sudo apt-get update
    sudo apt-get install nvidia-l4t-jetson-multimedia-api
    sudo apt-get install nvidia-cuda
```

[```Note```]: The cuda binding rebuild only when running ```cmake``` commands.
## [```Optional```] In order to install a new camera image with reduced size, use the following instructions

### 1. On camera
* Connect to camera via ssh (Either through ethenet connetion or wifi connection)
* Run the following command lines on the camera:
  ```console
    wget https://swdownloads.analog.com/cse/aditof/smart_camera/img_update/cam_update.tar
    tar -xvf cam_update.tar
    cd cam_update
    sudo bash update-script.sh
  ```
* After the automatic restart of the camera, open port number 8080 at the same ip address the camera had previously. (```<ip_addr>:8080```)
* Upload the following file via the ```Software upload``` interface: https://swdownloads.analog.com/cse/aditof/smart_camera/img_update/adi-jetson-nano-img_1.0.swu
* After uploading the file and rebooting the camera you can connect again via ssh, and run the following commands (the installation commands may take a few minutes):
```console
    sudo apt-get update
    sudo apt-get install nvidia-l4t-jetson-multimedia-api
    sudo apt-get install nvidia-cuda
```
* add ```nvcc``` to the macro PATH:
```console
export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}$ 
export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```
At this point the camera is ready to be used with cuda API.