% Example code to read in data files and save them as matrix files
% Christopher D. Yoder
% 08-17-2019
% NOTE: These decoding scripts only work with Gerrit's electronics package
% lines. 

clc
close all
clear

% from parsefile.m:
%
% data is a structure with the following fields:
%   data.GPGGA = the data contained in the GPGGA strings (GPS files)
%   data.GPRMC = the data contained in the GPRMC strings (GPS files)
%   data.AO = the data contained within the AO files (AO files)
%   data.GND = the data contained with the GPS2018 file (GND Station)
%
% GPGGA = [hr, mn, sc, latdd, londd, gpsfix, satnums, hdop, alt, geoidhgt, elpsdtme]
% GPRMC = [hr, mn, sc, latdd, londd, spd, magvar, elpsdtme]
% AO = [hr, mn, sc, ms, alt, ax, ay, az, wx, wy, wz, mx, my, mz, ...
%       ex, ey, ez, gx, gy, gz, temp, pres, humd, altalt, pitot, elpsdtme]
% GND2018 = [hr, mn, sc, ms, alt, ax, ay, az, wx, wy, wz, mx, my, mz, ...
%       ex, ey, ez, gx, gy, gz, temp, speed, gust, dirn, elpsdtme]
% GND2019 = [hr, mn, sc, ms, alt, ax, ay, az, wx, wy, wz, mx, my, mz, ...
%       ex, ey, ez, gx, gy, gz, temp, pres, humd, altalt, pitot, elpsdtme];

% % gondo data
% GPSdata = ParseFile(fullfile(pwd, 'Data', 'GONDO', 'GPSLOG00.TXT'), ...
%     'filetype', 'GPS', 'startline', 1010, 'linenum', 48960);
% save('gondoGPS.mat', 'GPSdata');
AOdata = ParseFile(fullfile(pwd, 'Data', 'GONDO', 'AODATA00.TXT'), ...
    'filetype', 'AO', 'startline', 1451, 'linenum', 137399);
% save('gondoAO_rev1.mat', 'AOdata');


% % sail data
% GPSdata = ParseFile(fullfile(pwd, 'Data', 'SAIL', 'GPSLOG00.TXT'), ...
%     'filetype', 'GPS', 'startline', 500);
% save('sailGPS.mat', 'GPSdata');
% AOdata = ParseFile(fullfile(pwd, 'Data', 'SAIL', 'AODATA00.TXT'), ...
%      'filetype', 'AO', 'startline', 1681);
% save('sailAO.mat', 'AOdata');

% % load cell
% % GetTension(fullfile(pwd, 'load'), 'field2');
% [ms, volts, tens] = ReadTension(fullfile(pwd, 'Data', 'load', 'newfile0.txt'));
% save('mooring.mat', 'ms', 'volts', 'tens');

% % OG ground station
% GPSdata = ParseFile(fullfile(pwd, 'Data', 'GND station', 'GPSLOG00.TXT'), ...
%     'filetype', 'GPS', 'startline', 483, 'linenum', 119194);
% save('gndOGGPS.mat', 'GPSdata');
% AOdata = ParseFile(fullfile(pwd, 'Data', 'GND station', 'AODATA00.TXT'), ...
%      'filetype', 'GND2018', 'startline', 1256, 'linenum', 264155);
% save('gndOGAO.mat', 'AOdata');

% % xtra ground station
% GPSdata = ParseFile(fullfile(pwd, 'Data', 'XTRA GND', 'GPSLOG00.TXT'), ...
%     'filetype', 'GPS', 'startline', 533, 'linenum', 119689);
% save('gndXTRAGPS.mat', 'GPSdata');
% AOdata = ParseFile(fullfile(pwd, 'Data', 'XTRA GND', 'AODATA00.TXT'), ...
%      'filetype', 'GND2019', 'startline', 1473, 'linenum', 334049);
% save('gndXTRAAO.mat', 'AOdata');


% disp
disp('Done!');