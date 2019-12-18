
%% The first devices identified will always have a USB connection
%% If there is only one USB device connected the following line
%% will create a video input based on it
vid = videoinput('aditofadapter');

%% If there are more cameras available over USB just select which one
%% should provide the video input (1, 2, 3, ...)
%%vid = videoinput('aditofadapter', 1);

preview(vid);
