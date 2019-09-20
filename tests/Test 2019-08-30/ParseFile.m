function [data] = ParseFile(filename, varargin)
% [data] = ParseFile(filename) returns the data contained in the text file
% filename. filename needs to start with either "GPS" or "AO."
%
% data is a structure with the following fields:
%   data.GPGGA = the data contained in the GPGGA strings (GPS files)
%   data.GPRMC = the data contained in the GPRMC strings (GPS files)
%   data.AO = the data contained within the AO files (AO files)
%   data.GND = the data contained with the GPS2018 file (GND Station)
%
% GPGGA consists of the following rows:
%   [hr, mn, sc, latdd, londd, gpsfix, satnums, hdop, alt, geoidhgt, elpsdtme]
%   where:
%       hr = hour
%       mn = minute
%       sc = second
%       latdd = latitude, in decimal degrees and N denoted +VE
%       londd = longitude, in decimal degrees and E denoted +VE
%       gpsfix = does the GPS have a fix? -1 = yes
%       satnums = number of satellites
%       hdop = Horizontal Dilution of Precision
%       alt = altitude, m
%       geoidhgt = height of the geoid, m
%       elpsdtme = time of day expressed in seconds since midnight
%
% GPRMC consists of the following rows:
%   [hr, mn, sc, latdd, londd, spd, magvar, elpsdtme]
%   where:
%       hr = hour
%       mn = minute
%       sc = second
%       latdd = latitude, in decimal degrees and N denoted +VE
%       londd = longitude, in decimal degrees and E denoted +VE
%       spd = ground speed, knots
%       magvar = magnetic variation from North, deg
%       elpsdtme = time of day expressed in seconds since midnight
%
% AO consists of the following:
%   [hr, mn, sc, ms, alt, ax, ay, az, wx, wy, wz, mx, my, mz, ...
%       ex, ey, ez, gx, gy, gz, temp, pres, humd, altalt, pitot, elpsdtme]
%   where:
%       hr = hour
%       mn = minute
%       sc = second
%       ms = millisecond
%       [ax, ay, az] = acceleration in x, y, z in m/s
%       [wx, wy, wz] = angular velocity in x, y, z in m/s
%       [mx, my, mz] = magnetic field in x, y, z in m/s
%       [ex, ey, ez] = euler angles about x, y, z in m/s
%       [gx, gy, gz] = gravity in x, y, z in m/s
%       temp = temperature, degC
%       humd = humidity, %
%       altalt = altitude, m
%       pitot = wind speed m/s
%       elpsdtme = time of day expressed in seconds since midnight
%       
% GND consists of the following:
%   [hr, mn, sc, ms, alt, ax, ay, az, wx, wy, wz, mx, my, mz, ...
%       ex, ey, ez, gx, gy, gz, temp, speed, gust, dirn, elpsdtme]
%   where:
%       hr = hour
%       mn = minute
%       sc = second
%       ms = millisecond
%       [ax, ay, az] = acceleration in x, y, z in m/s
%       [wx, wy, wz] = angular velocity in x, y, z in m/s
%       [mx, my, mz] = magnetic field in x, y, z in m/s
%       [ex, ey, ez] = euler angles about x, y, z in m/s
%       [gx, gy, gz] = gravity in x, y, z in m/s
%       temp = temperature, degC
%       speed = wind speed, m/s
%       gust = wind gusts, m/s
%       dirn = wind direction, degN
%       elpsdtme = time of day expressed in seconds since midnight
%
%
%
% [data] = ParseFile(filename, 'name', value) returns the data contained
% in filename and applies the desired name/value pair commands specified by
% the user. The choices for name are:
%   'filetype' | 'any' (default),  'GPS', 'AO', 'GND2018'
%   Used to specify which type of file format to use. 
%   'startline' | 1 (default)
%   The line to take as the starting time for real data. 
%
% References:
% http://aprs.gids.nl/nmea/#rmc

% Written by Christopher D. Yoder
% 11/18/18 
% For parsing Gerrit balloon/sail data files
% from email:
% GPSTime + (ms since last GPSTime) "ms," altitude : Acc.x, Acc.y, Acc.z, gyro.x, gyro.y, gyro.z, mag.x, mag.y, mag.z, eul.x, eul,y, eul.z, grav.x, grav.y, grav.z, temp, press, humid, altAltitude, pitot.
% Where:
% Acc = Acceleration (m/s^2)
% gyro = Angular Velocity (rad/s)
% mag = Magnetic Strength Vector (uT)
% eul = Euler angles (degrees)
% grav = Gravity Vector (m/s^2)
% temp = BME280 temperature (deg. C)
% press = BME280 pressure (Pascals)
% altAltitude - BME280 calculated altitude (meters)
% pitot - Pitot averaged reading
%
% Mod 1, CDY, 1-6-19
% Added Ground Station parsing for new Ground Station for 12-18-18 test
% Taken from code:
%   OtherflushTotal = OtherflushTotal + F("\n") + gpsTime + F(" + ") + gps_timeAddition + F(" ms, ") + altitude + F(" : ");
%   OtherflushTotal = OtherflushTotal + BN_acc.x() + F(",") + BN_acc.y() + F(",") + BN_acc.z();
%   OtherflushTotal = OtherflushTotal + F(",") + BN_gyro.x() + F(",") + BN_gyro.y() + F(",") + BN_gyro.z();
%   OtherflushTotal = OtherflushTotal + F(",") + BN_mag.x() + F(",") + BN_mag.y() + F(",") + BN_mag.z();
%   OtherflushTotal = OtherflushTotal + F(",") + BN_eul.x() + F(",") + BN_eul.y() + F(",") + BN_eul.z();
%   OtherflushTotal = OtherflushTotal + F(",") + BN_grav.x() + F(",") + BN_grav.y() + F(",") + BN_grav.z();
%   OtherflushTotal = OtherflushTotal + F(",") + IMU.getTemp() + F(",") + currentWindSpeed + F(",") + currentWindGust;
%   OtherflushTotal = OtherflushTotal + F(",") + currentWindDirection + F("%%");
% The total line is:
%   gpsTime + milli, alt : ax, ay, az, wx, wy, wz, mx, my, mz, ex, ey, ez, gx, gy, gz, temp, speed, gust, dir
% where:
%   gpsTime     = GPS UTC time
%   milli       = milliseconds
%   alt         = GPS altitude
%   a           = acceleration in x, y, z
%   w           = angular velocity in x, y, z
%   m           = magnetic fields in x, y, z
%   e           = euler angles in x, y, z
%   g           = gravity in x, y, z
%   temp        = temperature in degC
%   speed       = wind speed, in m/s
%   gust        = wind speed gusts, in m/s
%   dir         = wind direction, in deg from N (N = 0, E = 90)

% set default settings
FileFlag = NaN;   % GPS flag = 0, AO flag = 1
startLine = 1;  % 1 == start at the top of the file
dirnoff = 0;    % deg, offset between ground station wind vane and IMU
linenum = 1e5;  % limit to dataset

% set varargin
if isempty(varargin) ~= 1
    for i1 = 1:length(varargin)/2
        switch varargin{2*(i1 - 1) + 1}
                        
            case 'filetype'
                if strcmp(varargin{2*(i1 - 1) + 2}, 'GPS') == 1
                    FileFlag = 0;
                elseif strcmp(varargin{2*(i1 - 1) + 2}, 'AO') == 1
                    FileFlag = 1;
                elseif strcmp(varargin{2*(i1 - 1) + 2}, 'GND2018') == 1
                    FileFlag = 2;
                elseif strcmp(varargin{2*(i1 - 1) + 2}, 'GND2019') == 1
                    FileFlag = 3;
                else
                    error('filetype value is not a valid selection.');
                end
                
            case 'startline'
                if isnumeric(varargin{2*(i1 - 1) + 2}) ~= 1
                    error('startline is not a numeric value.');
                end
                if varargin{2*(i1 - 1) + 2} < 1
                    error('startline cannot be less than 1.');
                else
                    startLine = varargin{2*(i1 - 1) + 2};
                end
                
            case 'dirnoff'
                if isnumeric(varargin{2*(i1 - 1) + 2}) ~= 1
                    error('dirnoff is not numeric.');
                else
                    dirnoff = varargin{2*(i1 - 1) + 2};
                end
                
            case 'linenum'
                if isnumeric(varargin{2*(i1 - 1) + 2}) ~= 1
                    error('linenum is not numeric.');
                else
                    linenum = varargin{2*(i1 - 1) + 2};
                end
                
            otherwise
        end
    end
end

% determine file type
if isnan(FileFlag) == 1
    if strcmp(filename(1:2), 'GP') == 1
        FileFlag = 0;
    elseif strcmp(filename(1:2), 'AO') == 1
        FileFlag = 1; 
    else
        error('filename does not start with either "GPS" or "AO." Please use the "filetype" name/value pair to specify the file type.');
    end
end

% parse file
cntr1 = 1;
cntr2 = 1;
cntr3 = 1;
switch FileFlag 
    
    % GPS file
    case 0
        fid = fopen(filename, 'r');
        for j1 = 1:startLine
            newline = fgetl(fid);
        end
        exitFlag = 0;
        GPGGA = NaN(linenum, 11);
        GPRMC = NaN(linenum, 8);
        while exitFlag < 3
            % newline
            if newline == -1
                exitFlag = exitFlag - newline;
            else
                c1 = strsplit(newline, ',');
                % if strcmp(c1{1}, '$GPGGA') == 1 && length(c1) > 10
                try
                    if strcmp(c1{1}, '$GPGGA') == 1 && length(c1) >= 14
                        tmeraw = c1{2};
                        hr = str2double(tmeraw(1:2));
                        mn = str2double(tmeraw(3:4));
                        sc = str2double(tmeraw(5:end));
                        latraw = c1{3};
                        latdd = str2double(latraw(1:2)) + str2double(latraw(3:end))/60;
                        if strcmp(c1{4}, 'S') == 1
                            latdd = -latdd;
                        end
                        lonraw = c1{5};
                        londd = str2double(lonraw(1:3)) + str2double(lonraw(4:end))/60;
                        if strcmp(c1{6}, 'W') == 1
                            londd = -londd;
                        end
                        gpsfix = str2double(c1{7});
                        satnums = str2double(c1{8});
                        hdop = str2double(c1{9});
                        alt = str2double(c1{10});
                        geoidhgt = str2double(c1{12});
                        elpsdtme = hr*3600 + mn*60 + sc;
                        temparry = [hr, mn, sc, latdd, londd, gpsfix, satnums, hdop, alt, geoidhgt, elpsdtme];
                        % GPGGA = [GPGGA; temparry];
                        GPGGA(cntr1, :) = temparry;
                        cntr1 = cntr1 + 1;
                        % keyboard
                    
                    elseif strcmp(c1{1}, '$GPRMC') == 1 && length(c1) >= 11
                    % elseif strcmp(c1{1}, '$GPRMC') == 1 && length(c1) > 10
                        tmeraw = c1{2};
                        if length(tmeraw) > 3
                            hr = str2double(tmeraw(1:2));
                            mn = str2double(tmeraw(3:4));
                            sc = str2double(tmeraw(5:end));
                            latraw = c1{4};
                            latdd = str2double(latraw(1:2)) + str2double(latraw(3:end))/60;
                            if strcmp(c1{5}, 'S') == 1
                                latdd = -latdd;
                            end
                            lonraw = c1{6};
                            londd = str2double(lonraw(1:3)) + str2double(lonraw(4:end))/60;
                            if strcmp(c1{7}, 'W') == 1
                                londd = -londd;
                            end
                            spd = str2double(c1{8});
                            magvar = str2double(c1{9});
                            elpsdtme = hr*3600 + mn*60 + sc;
                            temparry = [hr, mn, sc, latdd, londd, spd, magvar, elpsdtme];
                            % GPRMC = [GPRMC; temparry];
                            GPRMC(cntr2, :) = temparry;
                            cntr2 = cntr2 + 1;
                        end   
                    else
                        % warning('Dunno what to do here.... skip?');
                        % newline
                        % c1
                        % keyboard
                    end
                catch
                    newline
                    % c1
                    % keyboard
                end
            end  
            newline = fgetl(fid);
        end
        fclose(fid);
        data.GPGGA = GPGGA;
        data.GPRMC = GPRMC;
    
    % AO file
    case 1
        fid = fopen(filename, 'r');
        for j1 = 1:startLine
            newline = fgetl(fid);
        end
        exitFlag = 0;
        AO = NaN(linenum, 26);
        cntr = 0;
        while exitFlag < 3
            % cntr = cntr + 1
            if newline == -1
                exitFlag = exitFlag - newline;
            else
                c1 = strsplit(newline, ',');    
                d1 = strsplit(c1{1});
                tmeraw = d1{1};
                if isempty(tmeraw) == 1
                    hr = 0;
                    mn = 0;
                    sc = 0;
                    
                else
                    hr = str2double(tmeraw(1:2));
                    mn = str2double(tmeraw(3:4));
                    sc = str2double(tmeraw(5:end));
                end
                % ms = str2double(d1{3});
                ms1 = c1{2};
                ms = str2double(ms1(1:end-2));
                if ms < 0
                    ms = -ms;
                end
                elpsdtme = hr*3600 + mn*60 + sc + ms/1000;
                % d2 = strsplit(c1{2}, ':');
                d2 = strsplit(c1{3}, ':');
                alt = str2double(d2{1});
                ax = str2double(d2{2});
                ay = str2double(c1{4});
                az = str2double(c1{5});
                wx = str2double(c1{6});
                wy = str2double(c1{7});
                wz = str2double(c1{8});
                mx = str2double(c1{9});
                my = str2double(c1{10});
                mz = str2double(c1{11});
                ex = str2double(c1{12});
                ey = str2double(c1{13});
                ez = str2double(c1{14});
                gx = str2double(c1{15});
                gy = str2double(c1{16});
                gz = str2double(c1{17});
                temp = str2double(c1{18});
                pres = str2double(c1{19});
                humd = str2double(c1{20});
                altalt = str2double(c1{21});
                try
                    % b1 = strsplit(c1{21}, '%%');
                    b1 = strsplit(c1{22}, '--');
                catch
                    keyboard
                end
                pitot = str2double(b1{1});
                temparry = [hr, mn, sc, ms, alt, ax, ay, az, ...
                    wx, wy, wz, mx, my, mz, ex, ey, ez, gx, gy, ...
                    gz, temp, pres, humd, altalt, pitot, elpsdtme];
                % AO = [AO; temparry];
                AO(cntr3, :) = temparry;
                cntr3 = cntr3 + 1;
            end
            newline = fgetl(fid);
            cntr = cntr + 1;
            if cntr > 1e6
                warning('Premature termination.');
                exitFlag = 5;
            end
        end
        fclose(fid);
        data.AO = AO;
        
    % Ground station 2018 file
    case 2
        fid = fopen(filename, 'r');
        for j1 = 1:startLine
            newline = fgetl(fid);
        end
        exitFlag = 0;
        GND = NaN(linenum, 25);
        cntr = 0;
        while exitFlag < 3
            if newline == -1
                exitFlag = exitFlag - newline;
            else
                try
                    c1 = strsplit(newline, ',');    
                    d1 = strsplit(c1{1});
                    tmeraw = d1{1};
                    if isempty(tmeraw) == 1
                        hr = 0;
                        mn = 0;
                        sc = 0;

                    else
                        hr = str2double(tmeraw(1:2));
                        mn = str2double(tmeraw(3:4));
                        sc = str2double(tmeraw(5:end));
                    end
                        ms = str2double(d1{3});
                    if ms < 0
                        ms = -ms;
                    end
                    elpsdtme = hr*3600 + mn*60 + sc + ms/1000;
                    d2 = strsplit(c1{2}, ':');
                    alt = str2double(d2{1});
                    ax = str2double(d2{2});
                    ay = str2double(c1{3});
                    az = str2double(c1{4});
                    wx = str2double(c1{5});
                    wy = str2double(c1{6});
                    wz = str2double(c1{7});
                    mx = str2double(c1{8});
                    my = str2double(c1{9});
                    mz = str2double(c1{10});
                    ex = str2double(c1{11});
                    ey = str2double(c1{12});
                    ez = str2double(c1{13});
                    gx = str2double(c1{14});
                    gy = str2double(c1{15});
                    gz = str2double(c1{16});
                    temp = str2double(c1{17});
                    sped = str2double(c1{18});
                    gust = str2double(c1{19});               
                    b1 = strsplit(c1{20}, '%%');
                    dirn = str2double(b1{1}) + dirnoff;
                    temparry = [hr, mn, sc, ms, alt, ax, ay, az, ...
                        wx, wy, wz, mx, my, mz, ex, ey, ez, gx, gy, ...
                        gz, temp, sped, gust, dirn, elpsdtme];
                    % GND = [GND; temparry];
                    GND(cntr3, :) = temparry;
                    cntr3 = cntr3 + 1;
                catch
                    keyboard
                end
            end
            newline = fgetl(fid);
            cntr = cntr + 1;
            if cntr > 5e5
                warning('Premature termination.');
                exitFlag = 5;
            end
        end
        fclose(fid);
        data.GND = GND;
        
    % Ground station 08-2019 flight
    case 3
        fid = fopen(filename, 'r');
        for j1 = 1:startLine
            newline = fgetl(fid);
        end
        exitFlag = 0;
        % GND = [];
        GND = NaN(linenum, 26);
        cntr = 0;
        while exitFlag < 3
            if newline == -1
                exitFlag = exitFlag - newline;
            else
                try
                    c1 = strsplit(newline, ',');    
                    d1 = strsplit(c1{1});
                    tmeraw = d1{1};
                    if isempty(tmeraw) == 1
                        hr = 0;
                        mn = 0;
                        sc = 0;

                    else
                        hr = str2double(tmeraw(1:2));
                        mn = str2double(tmeraw(3:4));
                        sc = str2double(tmeraw(5:end));
                    end
                        % ms1 = c1{2};
                        % ms = str2double(ms1(1:end-2));
                        % ms = str2double(c1{3});
                        % keyboard
                        ms1 = strsplit(c1{2}, 'M');
                        ms = str2double(ms1{1});
                    if ms < 0
                        ms = -ms;
                    end
                    elpsdtme = hr*3600 + mn*60 + sc + ms/1000;
                    % d2 = strsplit(c1{2}, ':');
                    d2 = strsplit(c1{3}, ':');
                    alt = str2double(d2{1});
                    ax = str2double(d2{2});
                    ay = str2double(c1{4});
                    az = str2double(c1{5});
                    wx = str2double(c1{6});
                    wy = str2double(c1{7});
                    wz = str2double(c1{8});
                    mx = str2double(c1{9});
                    my = str2double(c1{10});
                    mz = str2double(c1{11});
                    ex = str2double(c1{12});
                    ey = str2double(c1{13});
                    ez = str2double(c1{14});
                    gx = str2double(c1{15});
                    gy = str2double(c1{16});
                    gz = str2double(c1{17});
                    temp = str2double(c1{18});
                    pres = str2double(c1{19});
                    humd = str2double(c1{20});
                    altalt = str2double(c1{21});
                    try
                        % b1 = strsplit(c1{21}, '%%');
                        b1 = strsplit(c1{22}, '--');
                    catch
                        keyboard
                    end
                    pitot = str2double(b1{1});
%                     temparry = [hr, mn, sc, ms, alt, ax, ay, az, ...
%                         wx, wy, wz, mx, my, mz, ex, ey, ez, gx, gy, ...
%                         gz, temp, sped, gust, dirn, elpsdtme];
                    temparry = [hr, mn, sc, ms, alt, ax, ay, az, ...
                        wx, wy, wz, mx, my, mz, ex, ey, ez, gx, gy, ...
                        gz, temp, pres, humd, altalt, pitot, elpsdtme];
                    % GND = [GND; temparry];
                    GND(cntr3, :) = temparry;
                    cntr3 = cntr3 + 1;
                catch
                    keyboard
                end
            end
            newline = fgetl(fid);
            cntr = cntr + 1;
            if cntr > 5e5
                warning('Premature termination.');
                exitFlag = 5;
            end
        end
        fclose(fid);
        data.GND = GND;
        
    otherwise
        data = NaN;
end

end