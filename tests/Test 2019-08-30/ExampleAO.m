% Example code to read in data files and save them as matrix files
% Christopher D. Yoder
% 08-17-2019
% NOTE: These decoding scripts only work with Gerrit's electronics package
% lines. 
% 
% Modifications:
% 1. Copied to new directory by CDY, parsing new test data

% NOTE:
% 1. The code will scan lines in the text file and will skip lines which do
% not conform to the expected formatting. The skipped lines will be printed
% to the cmd line. 


clc
close all
clear

% % gondo data
% GPSdata = ParseFile(fullfile(pwd, 'Gondola1', 'GPSLOG00.TXT'), ...
%     'filetype', 'GPS', 'startline', 530, 'linenum', 63271);
% save('gondoGPS_1.mat', 'GPSdata');
% GPSdata = ParseFile(fullfile(pwd, 'Gondola2', 'GPSLOG00.TXT'), ...
%     'filetype', 'GPS', 'startline', 276, 'linenum', 49000);
% save('gondoGPS_2.mat', 'GPSdata');
% cGPSdata = ParseFile(fullfile(pwd, 'Gondola3', 'GPSLOG01.TXT'), ...
%     'filetype', 'GPS', 'startline', 1330, 'linenum', 26500);
% save('gondoGPS_3.mat', 'GPSdata');
% AOdata = ParseFile(fullfile(pwd, 'Gondola1', 'AODATA00.TXT'), ...
%     'filetype', 'AO', 'startline', 1462, 'linenum', 179000);
% save('gondoAO_1.mat', 'AOdata');
% AOdata = ParseFile(fullfile(pwd, 'Gondola2', 'AODATA00.TXT'), ...
%     'filetype', 'AO', 'startline', 1446, 'linenum', 138000);
% save('gondoAO_2.mat', 'AOdata');
AOdata = ParseFile(fullfile(pwd, 'Gondola3', 'AODATA01.TXT'), ...
    'filetype', 'AO', 'startline', 1473, 'linenum', 75000);
save('gondoAO_3.mat', 'AOdata');

% % sail data
% GPSdata1 = ParseFile(fullfile(pwd, 'Sail1', 'GPSLOG01.TXT'), ...
%     'filetype', 'GPS', 'startline', 536, 'linenum', 14100);
% GPSdata2 = ParseFile(fullfile(pwd, 'Sail1', 'GPSLOG02.TXT'), ...
%     'filetype', 'GPS', 'startline', 872, 'linenum', 42200);
% GPSdata3 = ParseFile(fullfile(pwd, 'Sail1', 'GPSLOG03.TXT'), ...
%     'filetype', 'GPS', 'startline', 330, 'linenum', 16700);
% GPGGA = [GPSdata1.GPGGA; GPSdata2.GPGGA; GPSdata3.GPGGA];   % assemble all data
% GPRMC = [GPSdata1.GPRMC; GPSdata2.GPRMC; GPSdata3.GPRMC];   % as one structure 
% GPSdata.GPGGA = GPGGA;                                      % and save for later
% GPSdata.GPRMC = GPRMC;                                      % use
% save('sailGPS_1.mat', 'GPSdata');
% GPSdata = ParseFile(fullfile(pwd, 'Sail2', 'GPSLOG00.TXT'), ...
%     'filetype', 'GPS', 'startline', 639, 'linenum', 56200);
% save('sailGPS_2.mat', 'GPSdata');
% GPSdata = ParseFile(fullfile(pwd, 'Sail3', 'GPSLOG00.TXT'), ...
%     'filetype', 'GPS', 'startline', 703, 'linenum', 31800);
% save('sailGPS_3.mat', 'GPSdata');
% AOdata1 = ParseFile(fullfile(pwd, 'Sail1', 'AODATA01.TXT'), ...
%      'filetype', 'AO', 'startline', 1651, 'linenum', 41100);
% AOdata2 = ParseFile(fullfile(pwd, 'Sail1', 'AODATA02.TXT'), ...
%      'filetype', 'AO', 'startline', 1694, 'linenum', 123900);
% AOdata3 = ParseFile(fullfile(pwd, 'Sail1', 'AODATA03.TXT'), ...
%      'filetype', 'AO', 'startline', 1688, 'linenum', 49200); 
% AOdata = [AOdata1.AO; AOdata2.AO; AOdata3.AO];   % assemble all data
% save('sailAO_1.mat', 'AOdata');
% AOdata = ParseFile(fullfile(pwd, 'Sail2', 'AODATA00.TXT'), ...
%      'filetype', 'AO', 'startline', 1699, 'linenum', 164800);
% save('sailAO_2.mat', 'AOdata');
% AOdata = ParseFile(fullfile(pwd, 'Sail3', 'AODATA00.TXT'), ...
%      'filetype', 'AO', 'startline', 1697, 'linenum', 93800);
% save('sailAO_3.mat', 'AOdata');


% % load cell - no files yet
% % GetTension(fullfile(pwd, 'load'), 'field2');
% [ms, volts, tens] = ReadTension(fullfile(pwd, 'Data', 'load', 'newfile0.txt'));
% save('mooring.mat', 'ms', 'volts', 'tens');

% % OG ground station - no data
% GPSdata = ParseFile(fullfile(pwd, 'Data', 'GND station', 'GPSLOG00.TXT'), ...
%     'filetype', 'GPS', 'startline', 483, 'linenum', 119194);
% save('gndOGGPS.mat', 'GPSdata');
% AOdata = ParseFile(fullfile(pwd, 'Data', 'GND station', 'AODATA00.TXT'), ...
%      'filetype', 'GND2018', 'startline', 1256, 'linenum', 264155);
% save('gndOGAO.mat', 'AOdata');

% xtra ground station
% GPSdata1 = ParseFile(fullfile(pwd, 'XTRA_GND_STN', 'GPSLOG01.TXT'), ...
%     'filetype', 'GPS', 'startline', 495, 'linenum', 890);
% GPSdata2 = ParseFile(fullfile(pwd, 'XTRA_GND_STN', 'GPSLOG02.TXT'), ...
%     'filetype', 'GPS', 'startline', 592, 'linenum', 193200);
% GPGGA = [GPSdata1.GPGGA; GPSdata2.GPGGA];   % assemble all data
% GPRMC = [GPSdata1.GPRMC; GPSdata2.GPRMC];   % as one structure 
% GPSdata.GPGGA = GPGGA;                                      % and save for later
% GPSdata.GPRMC = GPRMC;                                      % use
% save('gndXTRAGPS.mat', 'GPSdata');
% AOdata1 = ParseFile(fullfile(pwd, 'XTRA_GND_STN', 'AODATA01.TXT'), ...
%     'filetype', 'AO', 'startline', 1510, 'linenum', 2500);
% AOdata2 = ParseFile(fullfile(pwd, 'XTRA_GND_STN', 'AODATA02.TXT'), ...
%     'filetype', 'AO', 'startline', 1521, 'linenum', 539400);
% AOdata = [AOdata1.AO; AOdata2.AO];   % assemble all data
% save('gndXTRAAO.mat', 'AOdata');


% disp
disp('Done!');