%%%% Load the adaptor
adapts = imaqregister([pwd, '\aditofadapter.dll']); %when running on Linux load the aditofadapter.so
imaqreset;

%%%% Display information about the adaptor
imaqhwinfo
imaqfind
a = imaqhwinfo('aditofadapter')
a.DeviceInfo
a.DeviceInfo.DeviceName
clear a

%%%% Start a video stream using either USB or Ethernet
%% USB connection
vid = videoinput('aditofadapter');

%% Ethernet connection
%vid = videoinput('aditofadapter', 1, "<ip>"); % Replace the ip with the device on which the server runs

preview(vid);
pause
vid = [];

%%%% Run the image acquisition tool to see all the adaptor's attributes and play the video
imaqtool;
pause;

%%%% Unregister the adaptor
imaqregister([pwd, '\aditofadapter.dll'],'unregister');
imaqreset
imaqhwinfo