# Cuda binding for Smart Camera 3D
 Eqipment: 
 1. Analog Devices Smart Camera
 2. SSH connection to camera

## Stepps to prepare camera image:

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
    sudo apt-get install nvidia-l4t-multimedia-api
    sudo apt-get install nvidia-cuda
```
* add ```nvcc``` to the macro PATH:
```console
export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}$ 
export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```
At this point the camera is ready to be used with cuda API.