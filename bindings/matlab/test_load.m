%setenv('LD_LIBRARY_PATH',[getenv('LD_LIBRARY_PATH') ':/opt/glog/lib'])
%% Add
clc
adapts = imaqregister('build/bindings/matlab/aditofadapter.so');
imaqreset
imaqhwinfo
imaqfind
a = imaqhwinfo('aditofadapter')
a.DeviceInfo
a.DeviceInfo.DeviceName
clear a

vid = videoinput('aditofadapter');
preview(vid);
vid = [];
%% Remove
imaqregister('build/bindings/matlab/aditofadapter.so','unregister');
imaqreset
imaqhwinfo