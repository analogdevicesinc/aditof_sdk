
%% The second device will always be a generic ethernet device where the ip
%% is specified. Consider the example where there are no USB devices
%% the adapter will only display the generic ethernet device as available
%% In that case its id will be 1

vid = videoinput('aditofadapter', 1, "<ip>");
%% replace the ip with the device on which the server runs

preview(vid);
