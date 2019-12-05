## Network server

With the help of this server, a remote client can have access over the network to the API of the ADI Time of Flight sensor. The server needs to run on the target where the sensor is installed.

Limitations:
- Only one client can be connected to the server at a time.
- DragonBoard410c
  - For now, the aditof_sdk/examples/aditof-demo/config_pipe.sh needs to be ran before running the server (no need to run each time you re-run the server, unless you have rebooted the system).
  - Server can't work properly if at the same time the uvc-gadget is started and engaged with a client over USB.

## How to use

To start the server on the target run the following command:

    ./aditof-server
