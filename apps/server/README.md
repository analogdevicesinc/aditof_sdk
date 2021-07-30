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

## Troubleshooting

### Port error:

`[2021/07/29 01:23:08:8024] NOTICE: Creating Vhost 'default' port 5000, 1 protocols, IPv6 off`

`[2021/07/29 01:23:08:8027] ERR: ERROR on binding fd 6 to port 5000 (-1 98)`

`[2021/07/29 01:23:08:8028] ERR: init server failed`

`[2021/07/29 01:23:08:8029] ERR: Failed to create default vhost`

If this error occurs please modify the port value from the following locations:
* apps/server/server.cpp
* sdk/src/connections/network/network.cpp
