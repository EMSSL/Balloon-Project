% Example code to read in data files

clc
close all
clear

% % sail data
% GPSdata = ParseFile(fullfile(pwd, 'Data', 'SAIL', 'GPSLOG00.TXT'), ...
%     'filetype', 'GPS', 'startline', 500);
% save('sailGPS.mat', 'GPSdata');
% AOdata = ParseFile(fullfile(pwd, 'Data', 'SAIL', 'AODATA00.TXT'), ...
%      'filetype', 'AO', 'startline', 1681);
% save('sailAO.mat', 'AOdata');

% % gondo data
% GPSdata = ParseFile(fullfile(pwd, 'Data', 'GONDO', 'GPSLOG00.TXT'), ...
%     'filetype', 'GPS', 'startline', 1010);
% save('gondoGPS.mat', 'GPSdata');
% AOdata = ParseFile(fullfile(pwd, 'Data', 'GONDO', 'AODATA00.TXT'), ...
%     'filetype', 'AO', 'startline', 1451);
% save('gondoAO.mat', 'AOdata');

% load cell
% GetTension(fullfile(pwd, 'load'), 'field2');
[ms, volts, tens] = ReadTension(fullfile(pwd, 'Data', 'load', 'newfile0.txt'));
save('mooring.mat', 'ms', 'volts', 'tens');

% disp
disp('Done!');