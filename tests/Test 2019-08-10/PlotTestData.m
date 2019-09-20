function PlotTestData
% Example code to read in data files from the 08-10-2019 test
% Modeled after code used for the December plots
% Christopher D. Yoder
% 08-17-2019

% TO DO:
% 1. Sync Gondo, Sail, and GndStation to UTC
%   a. setup interpolation as needed
%   b. interpolate for GPS, AO data for all time
%   c. establish test times, find min time range (omit excess data)
% 2. plot the GPS data from the ground station(s) and determine offsets due
% to wandering error at each time step
%   a. estimate the corrections on gondo and sail using this method
%   b. plot the spatial walk of the ground station, find median/mean,
%   quantify wander
% 3. XYZ frame and times
%   a. Convert GPS to XYZ frame
%   b. Plot altitude vs time and select times at full deployment
%   c. Plot wind speed and direction to isolate like cases
%       i. Most important to find like conditions for three geometries
%   d. Plot IMU outputs, specifically roll and pitch
%       i. Estimate mean and std for roll and pitch angles
%       j. Plot transient response to wind gusts in roll and pitch
% 4. Plot wind speed/direction gradients during deployment/retrieval
% periods

clc
close all
clear

% % test times in seconds wrt 0 hrs
% NOTE: These are pulled by hand
ascent_float_descent_1 = [5.440, 5.493, 5.523, 5.560]*(10^4);
ascent_float_descent_2 = [5.560, 5.590, 5.616, 5.630]*(10^4);
ascent_float_descent_3 = [5.630, 5.643, 5.705, 5.735]*(10^4);
float_1 = ascent_float_descent_1(2:3);
float_2 = ascent_float_descent_2(2:3);
float_3 = ascent_float_descent_3(2:3);
GPS_fix = [4.9, 5.831]*(10^4);          % time to average GPS station values

% directories
dirsGPS = {'gndOGGPS.mat';
           'gondoGPS.mat';
           'sailGPS.mat';
           'gndXTRAGPS.mat'};
dirsAO = {'gndOGAO.mat';
           'gondoAO.mat';
           'sailAO.mat';
           'gndXTRAAO.mat'};

% magnetic declination for RDU for 12/18/18
% taken from: http://www.geomag.nrcan.gc.ca/calc/mdcal-en.php
% dec_deg = DMStoDD(9, 19.32, 0, 'W');
dec_deg = DMStoDD(0, 19.86, 0, 'W');    % 35.7685N, 78.6624W

% go get datas
GPSstruct = {};
AOstruct = {};
for i1 = 1:length(dirsGPS)
    s1 = load(dirsGPS{i1});
    switch i1
        case 1
            GPSstruct.OGGS = s1.GPSdata;
        case 2
            GPSstruct.Gond = s1.GPSdata;
        case 3
            GPSstruct.Sail = s1.GPSdata;
        case 4
            GPSstruct.XGND = s1.GPSdata;
    end    
end

% AO data
for i1 = 1:length(dirsAO)     % skip ground station since not sure parse
    s1 = load(dirsAO{i1});
    % keyboard
    switch i1
        case 1
            AOstruct.OGGS = s1.AOdata.GND;
        case 2
            AOstruct.Gond = s1.AOdata.AO;
        case 3
            AOstruct.Sail = s1.AOdata.AO;
        case 4
            AOstruct.XGND = s1.AOdata.GND;
    end
end

% ------------------------------------------------------------------------
%                        plot all GPS data - raw
% ------------------------------------------------------------------------
% PlotGPSTestALL(Gondo_GPS, Vane_GPS, Sail_GPS, Ground_GPS, Elapsed_1, Elapsed_2) 
% takes the data in Gondo, Vane, Sail, and Ground station and returns
% figures of the data. 
% INPUTS
% Gondo_GPS     = Gondola GPS structure
% Vane_GPS      = Vane GPS structure (can replace with OG ground station)
% Sail_GPS      = Sail GPS structure
% Ground_GPS    = Ground GPS structure (new ground station)
% Elapsed_1     = starting times for each new trial in elapsed seconds
% Elapsed_2     = starting times for each new trial in elapsed seconds
PlotGPSTestALL(GPSstruct.Gond, GPSstruct.OGGS, GPSstruct.Sail, GPSstruct.XGND, [], []);
subplot(3, 1, 1);
title('All data, raw');
legend({'Gondola', 'OG GND', 'Sail', 'New GND'}, 'location', ...
    'southwest', 'interpreter', 'latex');
subplot(3, 1, 2);
ylim([-78.664, -78.662]);
subplot(3, 1, 3);
ylim([0, 300]);

% keyboard


% ------------------------------------------------------------------------
%                 plot wind speed/direction data - floats
% ------------------------------------------------------------------------
% Returns a plot of the specified data from the gondola, vane, sail, and
% ground station. Varargin can be any of the following name/value pairs:
%   'timebounds' | [start, end] = used to plot data for a given interval of
%   start and end. The value is a two element vector of start and stop
%   times. 
%   'data' | 'accl', 'mag', 'angvel', 'grav', 'wind', 'euler', 'head'
%   Used to specify what data to plot. Default is acceleration. 
%   'rho' | 1.225,  density of the atmosphere at ground level
%   'magdec' | 0 deg, magnetic declination angle for flight location. 
%   '
%   
% INPUTS:
%   Gondo_AO    = gondola AO data matrix
%   Vane_AO     = Vane AO data matrix
%   NewSail_AO  = NewSail AO data matrix
%   Ground_AO   = Ground AO data matrix
% 
% References:
% https://aerocontent.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf

% wind speed/dirn - all
windspd = PlotAOData(AOstruct.Gond, AOstruct.OGGS, AOstruct.Sail, AOstruct.XGND, ...
    'data', 'wind', 'magdec', dec_deg, 'dircal', 'solve', ...
    'title', 'all', 'pitotcal', '8-10-19-Cal');
winddir = PlotAOData(AOstruct.Gond, AOstruct.OGGS, AOstruct.Sail, AOstruct.XGND, ...
    'data', 'head', 'magdec', dec_deg, 'dircal', 'solve', ...
    'title', 'all');

% keyboard

% wind speeds
float_1 = ascent_float_descent_1(2:3);  % positive
float_2 = ascent_float_descent_2(2:3);  % zero
float_3 = ascent_float_descent_3(2:3);  % negative
f1 = PlotAOData(AOstruct.Gond, AOstruct.OGGS, AOstruct.Sail, AOstruct.XGND, ...
    'data', 'head', 'magdec', dec_deg, 'dircal', 'solve', 'timebounds', ...
    float_1, 'title', 'positive', 'plots', 'none');
datastats1 = CalcStats(f1.Gondo(f1.TsGondo:f1.TeGondo));
f2 = PlotAOData(AOstruct.Gond, AOstruct.OGGS, AOstruct.Sail, AOstruct.XGND, ...
    'data', 'head', 'magdec', dec_deg, 'dircal', 'solve', 'timebounds', ...
    float_2, 'title', 'zero', 'plots', 'none');
datastats2 = CalcStats(f2.Gondo(f2.TsGondo:f2.TeGondo));
f3 = PlotAOData(AOstruct.Gond, AOstruct.OGGS, AOstruct.Sail, AOstruct.XGND, ...
    'data', 'head', 'magdec', dec_deg, 'dircal', 'solve', 'timebounds', ...
    float_3, 'title', 'negative', 'plots', 'none');
datastats3 = CalcStats(f3.Gondo(f3.TsGondo:f3.TeGondo));
% fprintf('%20s%20s%20s\n', 'Gondo heading stats:', 'Positive', 'Negative');
% fprintf('%20s%20.4f%20.4f\n', 'xbar = ', datastats1.xbar, datastats2.xbar);
% fprintf('%20s%20.4f%20.4f\n', 'median = ', datastats1.medi, datastats2.medi);
% fprintf('%20s%20.4f%20.4f\n', 'std = ', datastats1.sigm, datastats2.sigm);

keyboard

% plot wind speed - only floats
shortpos = PlotAOData(AOstruct.Gond, AOstruct.Vane, AOstruct.Sail, AOstruct.GndS, ...
    'data', 'wind', 'timebounds', test1, 'title', 'positive', 'plots', 'none');
data1G = shortpos.Gondo(shortpos.TsGondo:shortpos.TeGondo);         % bin values - bad
data1V = shortpos.Vane(shortpos.TsVane:shortpos.TeVane);            % bin values
data1S = shortpos.Sail(shortpos.TsSail:shortpos.TeSail);            % bin values
data1GND = shortpos.Ground(shortpos.TsGround:shortpos.TeGround);    % wind speed m/s
vane_speed1 = ConvertToWindSpeed(data1V, 539, 1.225);
sail_speed1 = ConvertToWindSpeed(data1S, 535, 1.225);
datastats1 = CalcStats(vane_speed1);
datastats2 = CalcStats(sail_speed1);
datastats3 = CalcStats(data1GND);

figure('Color', 'w');
hold on
plot(shortpos.Sail_tme(shortpos.TsSail:shortpos.TeSail), sail_speed1);
plot(shortpos.Ground_tme(shortpos.TsGround:shortpos.TeGround), data1GND);
plot(shortpos.Vane_tme(shortpos.TsVane:shortpos.TeVane), vane_speed1);
grid on
xlabel('Time [s]');
ylabel('Wind speed [m/s]');
legend({'Sail', 'Ground', 'Vane'});
title('positive');

shortneg = PlotAOData(AOstruct.Gond, AOstruct.Vane, AOstruct.Sail, AOstruct.GndS, ...
    'data', 'wind', 'timebounds', test2, 'title', 'positive', 'plots', 'none');
data1G = shortpos.Gondo(shortneg.TsGondo:shortneg.TeGondo);         % bin values - bad
data1V = shortpos.Vane(shortneg.TsVane:shortneg.TeVane);            % bin values
data1S = shortpos.Sail(shortneg.TsSail:shortneg.TeSail);            % bin values
data1GND = shortneg.Ground(shortneg.TsGround:shortneg.TeGround);    % wind speed m/s
vane_speed1 = ConvertToWindSpeed(data1V, 539, 1.225);
sail_speed1 = ConvertToWindSpeed(data1S, 535, 1.225);
datastats11 = CalcStats(vane_speed1);
datastats22 = CalcStats(sail_speed1);
datastats33 = CalcStats(data1GND);

fprintf('\n%20s%20s%20s\n', 'Vane speed stats:', 'Positive', 'Negative');
fprintf('%20s%20.4f%20.4f\n', 'xbar = ', datastats1.xbar, datastats11.xbar);
fprintf('%20s%20.4f%20.4f\n', 'median = ', datastats1.medi, datastats11.medi);
fprintf('%20s%20.4f%20.4f\n', 'std = ', datastats1.sigm, datastats11.sigm);

fprintf('\n%20s%20s%20s\n', 'Sail speed stats:', 'Positive', 'Negative');
fprintf('%20s%20.4f%20.4f\n', 'xbar = ', datastats2.xbar, datastats22.xbar);
fprintf('%20s%20.4f%20.4f\n', 'median = ', datastats2.medi, datastats22.medi);
fprintf('%20s%20.4f%20.4f\n', 'std = ', datastats2.sigm, datastats22.sigm);

fprintf('\n%20s%20s%20s\n', 'Ground speed stats:', 'Positive', 'Negative');
fprintf('%20s%20.4f%20.4f\n', 'xbar = ', datastats3.xbar, datastats33.xbar);
fprintf('%20s%20.4f%20.4f\n', 'median = ', datastats3.medi, datastats33.medi);
fprintf('%20s%20.4f%20.4f\n', 'std = ', datastats3.sigm, datastats33.sigm);

figure('Color', 'w');
hold on
plot(shortneg.Sail_tme(shortneg.TsSail:shortneg.TeSail), sail_speed1);
plot(shortneg.Ground_tme(shortneg.TsGround:shortneg.TeGround), data1GND);
plot(shortneg.Vane_tme(shortneg.TsVane:shortneg.TeVane), vane_speed1);
grid on
xlabel('Time [s]');
ylabel('Wind speed [m/s]');
legend({'Sail', 'Ground', 'Vane'});
title('negative');

keyboard

% ------------------------------------------------------------------------
%                        plot all GPS data - corrected
% ------------------------------------------------------------------------
% [lat0, lon0, alt0, errs] = dGPS(Station, mover, varargin) returns
% the nominal lat0, lon0, and alt0 of a stationary ground station in
% addition to the time errors of a moving GPS point (mover). 
%
% INPUT:
% Station   = a matrix of [time, lat, lon, alt]
% Mover     = a matrix of [time, lat, lon, alt]
%
% OUTPUT:
% lat0      = time average latitude of station
% lon0      = time average longitude of station
% alt0      = time average altitude of station
% errs      = array of [time, lat_err, lon_err, alt_err] in the Mover times
% newMove   = Mover data corrected for the errors
%
% References:
% https://racelogic.support/01VBOX_Automotive/01General_Information/Knowledge_Base/How_does_DGPS_(Differential_GPS)_work%3F
% https://www.e-education.psu.edu/geog160/node/1925
Station = [GPSstruct.XGND.GPGGA(:, 11), GPSstruct.XGND.GPGGA(:, 4), GPSstruct.XGND.GPGGA(:, 5), GPSstruct.XGND.GPGGA(:, 9)]; 
Gondo_mat = [GPSstruct.Gond.GPGGA(:, 11), GPSstruct.Gond.GPGGA(:, 4), GPSstruct.Gond.GPGGA(:, 5), GPSstruct.Gond.GPGGA(:, 9)]; 
% Vane_mat = [GPSstruct.Vane.GPGGA(:, 11), GPSstruct.Vane.GPGGA(:, 4), GPSstruct.Vane.GPGGA(:, 5), GPSstruct.Vane.GPGGA(:, 9)]; 
Sail_mat = [GPSstruct.Sail.GPGGA(:, 11), GPSstruct.Sail.GPGGA(:, 4), GPSstruct.Sail.GPGGA(:, 5), GPSstruct.Sail.GPGGA(:, 9)]; 
% [lat0, lon0, alt0, Gondo_dGPS, errs, stats] = dGPS(Station, Gondo_mat, 'plot', 'station');
[lat0, lon0, alt0, Gondo_dGPS, errs, stats] = dGPS(Station, Gondo_mat, 'plot', 'off');
% [~, ~, ~, Vane_dGPS, errs, stats] = dGPS(Station, Vane_mat, 'plot', 'off');
[~, ~, ~, Sail_dGPS, errs, stats] = dGPS(Station, Sail_mat, 'plot', 'off');
GndS_dGPS = NaN*Station;                    % create matrix for ground station positions
GndS_dGPS(:, 1) = Station(:, 1);
GndS_dGPS(:, 2) = 0*Station(:, 2) + lat0;
GndS_dGPS(:, 3) = 0*Station(:, 3) + lon0;
GndS_dGPS(:, 4) = 0*Station(:, 4) + alt0;

keyboard

% ------------------------------------------------------------------------
%                        plot XYZ data - floats
% ------------------------------------------------------------------------
% plot the GPS XYZ data for only the small times
postest = GetXYZTest(Gondo_dGPS, Vane_dGPS, Sail_dGPS, GndS_dGPS, ...
    Elapsed_1, 'titles', 'positive', 'lat0', lat0, 'lon0', lon0, ...
    'input', 'vec', 'legendloc', 'northwest', 'figures', 'none');
negtest = GetXYZTest(Gondo_dGPS, Vane_dGPS, Sail_dGPS, GndS_dGPS, ...
    Elapsed_2, 'titles', 'negtive', 'lat0', lat0, 'lon0', lon0, ...
    'input', 'vec', 'legendloc', 'northwest', 'figures', 'none');

% % plot wind speed - ALL
% % PlotAOData(AOstruct.Gond, AOstruct.Vane, AOstruct.Sail, AOstruct.GndS, 'data', 'wind', 'pitotcal', 'generic', 'timebounds', Elapsed_1(1:2))
% % PlotAOData(AOstruct.Gond, AOstruct.Vane, AOstruct.Sail, AOstruct.GndS, 'data', 'wind', 'pitotcal', 'generic')
% GetXYZTest(Gondo_dGPS, Vane_dGPS, Sail_dGPS, GndS_dGPS, ...
%     , 'titles', 'positive', 'lat0', lat0, 'lon0', lon0, ...
%     'input', 'vec', 'legendloc', 'northwest');
% PlotAOData(AOstruct.Gond, AOstruct.Vane, AOstruct.Sail, AOstruct.GndS, 'data', 'wind')
% % PlotAOData(AOstruct.Gond, AOstruct.Vane, AOstruct.Sail, AOstruct.GndS, 'data', 'wind', 'pitotcal', '1-9-19-Cal')



    





% plot the GPS XYZ data for only the small times
postest = GetXYZTest(Gondo_dGPS, Vane_dGPS, Sail_dGPS, GndS_dGPS, ...
    test1, 'titles', 'positive', 'lat0', lat0, 'lon0', lon0, ...
    'input', 'vec', 'legendloc', 'northwest');
negtest = GetXYZTest(Gondo_dGPS, Vane_dGPS, Sail_dGPS, GndS_dGPS, ...
    test2, 'titles', 'negtive', 'lat0', lat0, 'lon0', lon0, ...
    'input', 'vec', 'legendloc', 'northwest');

% fix axis bounds/labels
p3 = postest.fig2;
xlimmin = min([postest.xlimMIN2, negtest.xlimMIN2]);
xlimmax = max([postest.xlimMAX2, negtest.xlimMAX2]);
ylimmin = min([postest.ylimMIN2, negtest.ylimMIN2]);
ylimmax = max([postest.ylimMAX2, negtest.ylimMAX2]);
p3.CurrentAxes.XLim = [xlimmin, xlimmax];
p3.CurrentAxes.YLim = [ylimmin, ylimmax];
p5 = negtest.fig2;
p5.CurrentAxes.XLim = [xlimmin, xlimmax];
p5.CurrentAxes.YLim = [ylimmin, ylimmax];


% keyboard


% % make a movie
% gondo = postest.Gondo;
% vane = postest.Vane;
% sail = postest.Sail;
% glo_tme = linspace(gondo(1, 1), gondo(end, 1), 501);
% gondo_X = interp1(gondo(:, 1), gondo(:, 2), glo_tme);
% gondo_Y = interp1(gondo(:, 1), gondo(:, 3), glo_tme);
% vane_X = interp1(vane(:, 1), vane(:, 2), glo_tme);
% vane_Y = interp1(vane(:, 1), vane(:, 3), glo_tme);
% sail_X = interp1(sail(:, 1), sail(:, 2), glo_tme);
% sail_Y = interp1(sail(:, 1), sail(:, 3), glo_tme);
% close all
% daspect([1, 1, 1]);
% lgnd = {'Balloon', 'Vane', 'Sail', 'Ground'};
% for i1 = 1:length(glo_tme)
%     plot(gondo_X(i1), gondo_Y(i1), 'o', 'MarkerSize', 12);
%     hold on
%     plot(vane_X(i1), vane_Y(i1), 's');
%     plot(sail_X(i1), sail_Y(i1), '+');
%     plot(0, 0, 'kx', 'MarkerSize', 12);
%     grid off
%     xlabel('X Displacement [m]');
%     ylabel('Y Displacement [m]');
%     hold off
%     xlim([-80, 0]);
%     ylim([-27.7548, 52.2452]);
%     title(sprintf('Time = %8.2f seconds', glo_tme(i1)));
%     legend(lgnd, 'location', 'northwest');
%     pause(0.1);
% end
% 
% keyboard



% % acceleration
% test1 = [77300, Elapsed_1(2)];  % second half of flight, 
% test2 = [78410, 78630];  % second half of flight, 
% shortpos = PlotAOData(AOstruct.Gond, AOstruct.Vane, AOstruct.Sail, AOstruct.GndS, ...
%     'data', 'accl', 'timebounds', test1, 'title', 'positive');
% shortneg = PlotAOData(AOstruct.Gond, AOstruct.Vane, AOstruct.Sail, AOstruct.GndS, ...
%     'data', 'accl', 'timebounds', test2, 'title', 'negative');


% angular velocity
test1 = [77300, Elapsed_1(2)];  % second half of flight, 
test2 = [78410, 78630];  % second half of flight, 
shortpos = PlotAOData(AOstruct.Gond, AOstruct.Vane, AOstruct.Sail, AOstruct.GndS, ...
    'data', 'angvel', 'timebounds', test1, 'title', 'positive');
shortneg = PlotAOData(AOstruct.Gond, AOstruct.Vane, AOstruct.Sail, AOstruct.GndS, ...
    'data', 'angvel', 'timebounds', test2, 'title', 'negative');

% plot singles
close all
TsSail = shortpos.TsSail;
TeSail = shortpos.TeSail;
T0 = shortpos.Sail_tme(TsSail);
figure('Color', 'w');
hold on
plot(shortpos.Sail_tme(TsSail:TeSail) - T0, shortpos.Sail(TsSail:TeSail, 2));   % roll
plot(shortpos.Sail_tme(TsSail:TeSail) - T0, shortpos.Sail(TsSail:TeSail, 3));   % pitch
plot(shortpos.Sail_tme(TsSail:TeSail) - T0, shortpos.Sail(TsSail:TeSail, 1));   % yaw
xlabel('Elapsed time [s]', 'interpreter', 'latex');
ylabel('Angular velocity', 'interpreter', 'latex');
legend({'Roll', 'Pitch', 'Yaw'}, 'interpreter', 'latex');
grid on
title('positive', 'interpreter', 'latex');

indx = [2, 3, 1];
lgnd = {'Roll', 'Pitch', 'Yaw'};
for i1 = 1:3
    figure('Color', 'w');
    plot(shortpos.Sail_tme(TsSail:TeSail) - T0, shortpos.Sail(TsSail:TeSail, indx(i1)));   % roll
    xlabel('Elapsed time [s]', 'interpreter', 'latex');
    ylabel(sprintf('%s Rate [deg/s]', lgnd{i1}), 'interpreter', 'latex');
    title('positive', 'interpreter', 'latex');
    grid on
    switch i1
        case 1
            xlim([100, 120]);
            set(gca, 'ytick', -6:1:8); 
        case 2
            xlim([100, 160]);
            set(gca, 'xtick', 100:5:160);
        case 3
            set(gca, 'xtick', 44:2:80);
            set(gca, 'ytick', -6:1:6);
            xlim([45, 80]);
    end
            
end
% keyboard

% FFT positive
indx = [2, 3, 1];
figure('Color', 'w')
for i1 = 1:3
    subplot(3, 1, i1);
    t1 = shortpos.Sail_tme(TsSail:TeSail) - T0;
    dt = mean(t1(2:end) - t1(1:end - 1));
    t2 = -t1(end)/2:dt:t1(end)/2;
    x1 = shortpos.Sail(TsSail:TeSail, indx(i1));
    y1 = fft(x1);
    plot(t2, abs(fftshift(y1)));
    grid on
    xlim([-10, 10]);
end
subplot(3, 1, 1); ylabel('Roll', 'interpreter', 'latex'); title('positive', 'interpreter', 'latex');
subplot(3, 1, 2); ylabel('Pitch', 'interpreter', 'latex');
subplot(3, 1, 3); ylabel('Yaw', 'interpreter', 'latex');    
xlabel('Frequency');





    

figure('Color', 'w');
TsSail = shortneg.TsSail;
TeSail = shortneg.TeSail;
T0 = shortneg.Sail_tme(TsSail);
hold on
plot(shortneg.Sail_tme(TsSail:TeSail) - T0, shortneg.Sail(TsSail:TeSail, 2));   % roll
plot(shortneg.Sail_tme(TsSail:TeSail) - T0, shortneg.Sail(TsSail:TeSail, 3));   % pitch
plot(shortneg.Sail_tme(TsSail:TeSail) - T0, shortneg.Sail(TsSail:TeSail, 1));   % yaw
xlabel('Elapsed time [s]', 'interpreter', 'latex');
ylabel('Angular velocity', 'interpreter', 'latex');
legend({'Roll', 'Pitch', 'Yaw'}, 'interpreter', 'latex');
grid on
title('negative', 'interpreter', 'latex');

indx = [2, 3, 1];
lgnd = {'Roll', 'Pitch', 'Yaw'};
for i1 = 1:3
    figure('Color', 'w');
    plot(shortneg.Sail_tme(TsSail:TeSail) - T0, shortneg.Sail(TsSail:TeSail, indx(i1)));   % roll
    xlabel('Elapsed time [s]', 'interpreter', 'latex');
    ylabel(sprintf('%s Rate [deg/s]', lgnd{i1}), 'interpreter', 'latex');
    title('negative', 'interpreter', 'latex');
    grid on
    switch i1
        case 1
            set(gca, 'xtick', 40:4:90)
            xlim([40, 90])
        case 2
            set(gca, 'xtick', 50:4:100)
            xlim([50, 100])
        case 3
            xlim([100, 120]);
            set(gca, 'xtick', 100:2:120);
    end
            
end
keyboard

% FFT positive
indx = [2, 3, 1];
figure('Color', 'w')
for i1 = 1:3
    subplot(3, 1, i1);
    t1 = shortneg.Sail_tme(TsSail:TeSail) - T0;
    dt = mean(t1(2:end) - t1(1:end - 1));
    t2 = -t1(end)/2:dt:t1(end)/2;
    x1 = shortneg.Sail(TsSail:TeSail, indx(i1));
    y1 = fft(x1);
    plot(t2, abs(fftshift(y1)));
    grid on
    xlim([-10, 10]);
end
subplot(3, 1, 1); ylabel('Roll', 'interpreter', 'latex'); title('negative', 'interpreter', 'latex');
subplot(3, 1, 2); ylabel('Pitch', 'interpreter', 'latex');
subplot(3, 1, 3); ylabel('Yaw', 'interpreter', 'latex');    
xlabel('Frequency');




% % euler angles - not calibrated
% test1 = [77300, Elapsed_1(2)];  % second half of flight, 
% test2 = [78410, 78630];  % second half of flight, 
% shortpos = PlotAOData(AOstruct.Gond, AOstruct.Vane, AOstruct.Sail, AOstruct.GndS, ...
%     'data', 'euler', 'timebounds', test1, 'title', 'positive');
% shortneg = PlotAOData(AOstruct.Gond, AOstruct.Vane, AOstruct.Sail, AOstruct.GndS, ...
%     'data', 'euler', 'timebounds', test2, 'title', 'negative');







% % write to text files for Lanzi
% % shortpos
% % shortneg
% shortpos = PlotAOData(AOstruct.Gond, AOstruct.Vane, AOstruct.Sail, AOstruct.GndS, ...
%     'data', 'all', 'timebounds', test1, 'title', 'positive');
% shortneg = PlotAOData(AOstruct.Gond, AOstruct.Vane, AOstruct.Sail, AOstruct.GndS, ...
%     'data', 'all', 'timebounds', test2, 'title', 'negative');
% 
% % keyboard
% close all
% bigvec = MakeMatrix(postest, shortpos, 'gondo');
% WriteToTextFile(bigvec, 'gondo', 'Gondo_pos.txt');
% bigvec = MakeMatrix(postest, shortpos, 'vane');
% WriteToTextFile(bigvec, 'vane', 'Vane_pos.txt');
% bigvec = MakeMatrix(postest, shortpos, 'sail');
% WriteToTextFile(bigvec, 'sail', 'Sail_pos.txt');
% bigvec = MakeMatrix(postest, shortpos, 'ground');
% WriteToTextFile(bigvec, 'ground', 'Ground_pos.txt');
% 
% bigvec = MakeMatrix(negtest, shortneg, 'gondo');
% WriteToTextFile(bigvec, 'gondo', 'Gondo_neg.txt');
% bigvec = MakeMatrix(negtest, shortneg, 'vane');
% WriteToTextFile(bigvec, 'vane', 'Vane_neg.txt');
% bigvec = MakeMatrix(negtest, shortneg, 'sail');
% WriteToTextFile(bigvec, 'sail', 'Sail_neg.txt');
% bigvec = MakeMatrix(negtest, shortneg, 'ground');
% WriteToTextFile(bigvec, 'ground', 'Ground_neg.txt');
% keyboard


% ending things
disp(' ');
disp('Done!');
keyboard

end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function [x, y, z] = GPStoXYZ(lat, lon, alt, lat0, lon0)
% converts lat, lon, alt to x, y, z positions. makes the following
% assumptions:
%    1. The earth is perfectly spherical and of constant radius 6378m
%    2. The starting point specified by lat0, lon0
%    3. lat, lon are given in deg. alt is given in meters.

% errors
if isnumeric(lat) + isnumeric(lon) + isnumeric(alt) ~= 3
    error('All inputs must be numeric.');
end

% convert
R = 6378000;    % radius of earth
x = NaN*lon;
y = NaN*lat;
z = NaN*alt;
for i1 = 1:length(x)
    z(i1) = alt(i1);
    x(i1) = (pi/180)*(lon(i1) - lon0)*(R + z(i1));
    y(i1) = (pi/180)*(lat(i1) - lat0)*(R + z(i1));
end

end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function [testdata] = GetXYZTest(Gondo, Vane, Sail, Ground, timebounds, varargin)
% function to extract a particular test data set and plot it pretty
% Gondo = GPS gondola data
% Vane  = GPS vane data
% Sail = GPS sail data
% timebounds = start and end time of the test given in elapsed seconds from
%               midnight
% varargin = plotting stuffs

% errors
if length(timebounds) ~= 2
    error('timebounds needs to be a two element vector.');
end
if isnumeric(timebounds) ~= 1
    error('timebounds needs to be a numeric array.');
end

% handle inputs
figflags = 0;     % 'one' default
titlestr = ' ';
lat0 = NaN;
lon0 = NaN;
latflag = 0;
inputflag = 0;      % denotes structures
legendloc = 'best';
if isempty(varargin) ~= 1
    for i1 = 1:length(varargin)/2
        arg1 = varargin{2*(i1 - 1) + 1};
        arg2 = varargin{2*(i1 - 1) + 2};
        switch arg1
            case 'figures'
                if strcmp(arg2, 'all') == 1
                    figflags = 1;
                elseif strcmp(arg2, 'one') == 1
                    figflags = 0;
                elseif strcmp(arg2, 'none') == 1
                    figflags = -1;
                else
                    error('Input for figures not recognized.');
                end
            case 'titles'
                titlestr = arg2;
            case 'lat0'
                lat0 = arg2;
                latflag = 1;
            case 'lon0'
                lon0 = arg2;
            case 'input'
                if strcmp(arg2, 'struct') == 1
                    inputflag = 0;
                elseif strcmp(arg2, 'vec') == 1
                    inputflag = 1;
                elseif strcmp(arg2, 'mat') == 1
                    inputflag = 2;
                else
                    error('Input is not one of the following: struct, vec, or mat.');
                end
            case 'legendloc'
                legendloc = arg2;
            otherwise
        end
    end
end

if isnan(lat0) ~= isnan(lon0)
    error('Gotta change both lat0 and lon0 or neither.');
end
                   

% pull data for each test
switch inputflag
    case 0
        GGA_Gondo_lat = Gondo.GPGGA(:, 4);  % lat
        GGA_Gondo_lon = Gondo.GPGGA(:, 5);  % lon
        GGA_Gondo_alt = Gondo.GPGGA(:, 9);  % alt
        GGA_Gondo_tme = Gondo.GPGGA(:, 11); % elapsed time
        GGA_Vane_lat = Vane.GPGGA(:, 4);  % lat
        GGA_Vane_lon = Vane.GPGGA(:, 5);  % lon
        GGA_Vane_alt = Vane.GPGGA(:, 9);  % alt
        GGA_Vane_tme = Vane.GPGGA(:, 11); % elapsed time
        GGA_Sail_lat = Sail.GPGGA(:, 4);  % lat
        GGA_Sail_lon = Sail.GPGGA(:, 5);  % lon
        GGA_Sail_alt = Sail.GPGGA(:, 9);  % alt
        GGA_Sail_tme = Sail.GPGGA(:, 11); % elapsed time
        GGA_Ground_lat = Ground.GPGGA(:, 4);  % lat
        GGA_Ground_lon = Ground.GPGGA(:, 5);  % lon
        GGA_Ground_alt = Ground.GPGGA(:, 9);  % alt
        GGA_Ground_tme = Ground.GPGGA(:, 11); % elapsed time
        
    case 1
        GGA_Gondo_lat = Gondo(:, 2);  % lat
        GGA_Gondo_lon = Gondo(:, 3);  % lon
        GGA_Gondo_alt = Gondo(:, 4);  % alt
        GGA_Gondo_tme = Gondo(:, 1);    % elapsed time
        GGA_Vane_lat = Vane(:, 2);  % lat
        GGA_Vane_lon = Vane(:, 3);  % lon
        GGA_Vane_alt = Vane(:, 4);  % alt
        GGA_Vane_tme = Vane(:, 1); % elapsed time
        GGA_Sail_lat = Sail(:, 2);  % lat
        GGA_Sail_lon = Sail(:, 3);  % lon
        GGA_Sail_alt = Sail(:, 4);  % alt
        GGA_Sail_tme = Sail(:, 1); % elapsed time
        GGA_Ground_lat = Ground(:, 2);  % lat
        GGA_Ground_lon = Ground(:, 3);  % lon
        GGA_Ground_alt = Ground(:, 4);  % alt
        GGA_Ground_tme = Ground(:, 1); % elapsed time
        
    case 2
        error('Not setup yet. Use either struct or vec.');
end

% get test data
indx1 = sum(GGA_Gondo_tme <= timebounds(1));
indx2 = sum(GGA_Gondo_tme < timebounds(2));
Gondo_tme = GGA_Gondo_tme(indx1:indx2);
Gondo_lat = GGA_Gondo_lat(indx1:indx2);
Gondo_lon = GGA_Gondo_lon(indx1:indx2);
Gondo_alt = GGA_Gondo_alt(indx1:indx2);
if latflag == 0
    lat0 = Gondo_lat(1);
    lon0 = Gondo_lon(1);
end
% [Gondo_x, Gondo_y, Gondo_z] = GPStoXYZ(Gondo_lat, Gondo_lon, Gondo_alt, Gondo_lat(1), Gondo_lon(1));
[Gondo_x, Gondo_y, Gondo_z] = GPStoXYZ(Gondo_lat, Gondo_lon, Gondo_alt, lat0, lon0);
testdata.Gondo = [Gondo_tme, Gondo_x, Gondo_y, Gondo_z];
testdata.GondoLLA = [Gondo_tme, Gondo_lat, Gondo_lon, Gondo_alt];

indx1 = sum(GGA_Vane_tme <= timebounds(1));
indx2 = sum(GGA_Vane_tme < timebounds(2));
Vane_tme = GGA_Vane_tme(indx1:indx2);
Vane_lat = GGA_Vane_lat(indx1:indx2);
Vane_lon = GGA_Vane_lon(indx1:indx2);
Vane_alt = GGA_Vane_alt(indx1:indx2);
if latflag == 0
    lat0 = Vane_lat(1);
    lon0 = Vane_lon(1);
end
% [Vane_x, Vane_y, Vane_z] = GPStoXYZ(Vane_lat, Vane_lon, Vane_alt, Vane_lat(1), Vane_lon(1));
[Vane_x, Vane_y, Vane_z] = GPStoXYZ(Vane_lat, Vane_lon, Vane_alt, lat0, lon0);
testdata.Vane = [Vane_tme, Vane_x, Vane_y, Vane_z];
testdata.VaneLLA = [Vane_tme, Vane_lat, Vane_lon, Vane_alt];

indx1 = sum(GGA_Sail_tme <= timebounds(1));
indx2 = sum(GGA_Sail_tme < timebounds(2));
Sail_tme = GGA_Sail_tme(indx1:indx2);
Sail_lat = GGA_Sail_lat(indx1:indx2);
Sail_lon = GGA_Sail_lon(indx1:indx2);
Sail_alt = GGA_Sail_alt(indx1:indx2);
if latflag == 0
    lat0 = Sail_lat(1);
    lon0 = Sail_lon(1);
end
% [Sail_x, Sail_y, Sail_z] = GPStoXYZ(Sail_lat, Sail_lon, Sail_alt, Sail_lat(1), Sail_lon(1));
[Sail_x, Sail_y, Sail_z] = GPStoXYZ(Sail_lat, Sail_lon, Sail_alt, lat0, lon0);
testdata.Sail = [Sail_tme, Sail_x, Sail_y, Sail_z];
testdata.SailLLA = [Sail_tme, Sail_lat, Sail_lon, Sail_alt];

indx1 = sum(GGA_Ground_tme <= timebounds(1));
indx2 = sum(GGA_Ground_tme < timebounds(2));
Ground_tme = GGA_Ground_tme(indx1:indx2);
Ground_lat = GGA_Ground_lat(indx1:indx2);
Ground_lon = GGA_Ground_lon(indx1:indx2);
Ground_alt = GGA_Ground_alt(indx1:indx2);
if latflag == 0
    lat0 = Ground_lat(1);
    lon0 = Ground_lon(1);
end
% [Ground_x, Ground_y, Ground_z] = GPStoXYZ(Ground_lat, Ground_lon, Ground_alt, Ground_lat(1), Ground_lon(1));
[Ground_x, Ground_y, Ground_z] = GPStoXYZ(Ground_lat, Ground_lon, Ground_alt, lat0, lon0);
testdata.Ground = [Ground_tme, Ground_x, Ground_y, Ground_z];
testdata.GroundLLA = [Ground_tme, Ground_lat, Ground_lon, Ground_alt];

% plot all the plots
if figflags ~= -1
    indx = 3;
    xlimMIN1 = NaN(1, indx);
    xlimMAX1 = NaN(1, indx);
    xlimMIN2 = NaN;
    xlimMAX2 = NaN;
    ylimMIN1 = NaN(1, indx);
    ylimMAX1 = NaN(1, indx);
    ylimMIN2 = NaN;
    ylimMAX2 = NaN;
    if figflags == 0
        figure('Color', 'w');
    end
    fig1 = gcf;
    subplot(3, 1, 1);
    hold on
    p1 = plot(Gondo_tme - Gondo_tme(1), Gondo_y);
    p2 = plot(Vane_tme - Vane_tme(1), Vane_y);
    p3 = plot(Sail_tme - Sail_tme(1), Sail_y);
    p4 = plot(Ground_tme - Ground_tme(1), Ground_y);
    ylabel('Y [m]');
    grid on
    title(titlestr);
    legend([p1, p2, p3, p4], {'Gondo', 'Vane', 'Sail', 'Ground'}, 'location', legendloc)
    xlimMIN1(1) = min(xlim);
    xlimMAX1(1) = max(xlim);
    ylimMIN1(1) = min(ylim);
    ylimMAX1(1) = max(ylim);

    subplot(3, 1, 2);
    hold on
    p1 = plot(Gondo_tme - Gondo_tme(1), Gondo_x);
    p2 = plot(Vane_tme - Vane_tme(1), Vane_x);
    p3 = plot(Sail_tme - Sail_tme(1), Sail_x);
    p4 = plot(Ground_tme - Ground_tme(1), Ground_x);
    ylabel('X [m]');
    grid on
    xlimMIN1(2) = min(xlim);
    xlimMAX1(2) = max(xlim);
    ylimMIN1(2) = min(ylim);
    ylimMAX1(2) = max(ylim);

    subplot(3, 1, 3);
    hold on
    p1 = plot(Gondo_tme - Gondo_tme(1), Gondo_z);
    p2 = plot(Vane_tme - Vane_tme(1), Vane_z);
    p3 = plot(Sail_tme - Sail_tme(1), Sail_z);
    p4 = plot(Ground_tme - Ground_tme(1), Ground_z);
    ylabel('Z [m]');
    xlabel('Time [s]');
    grid on
    xlimMIN1(3) = min(xlim);
    xlimMAX1(3) = max(xlim);
    ylimMIN1(3) = min(ylim);
    ylimMAX1(3) = max(ylim);

    figure('Color', 'w');
    fig2 = gcf;
    hold on
    p1 = plot(Gondo_x, Gondo_y);
    p2 = plot(Vane_x, Vane_y);
    p3 = plot(Sail_x, Sail_y);
    p4 = plot(Ground_x, Ground_y);
    xlabel('X [m]');
    ylabel('Y [m]');
    grid on
    title(titlestr);
    legend([p1, p2, p3, p4], {'Gondo', 'Vane', 'Sail', 'Ground'}, 'location', legendloc)
    xlimMIN2 = min(xlim);
    xlimMAX2 = max(xlim);
    ylimMIN2 = min(ylim);
    ylimMAX2 = max(ylim);

    % assign figure handles
    testdata.fig1 = fig1;
    testdata.fig2 = fig2;
    testdata.xlimMIN1 = xlimMIN1;
    testdata.xlimMAX1 = xlimMAX1;
    testdata.ylimMIN1 = ylimMIN1;
    testdata.ylimMAX1 = ylimMAX1;
    testdata.xlimMIN2 = xlimMIN2;
    testdata.xlimMAX2 = xlimMAX2;
    testdata.ylimMIN2 = ylimMIN2;
    testdata.ylimMAX2 = ylimMAX2;
end
    
end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function PlotGPSTestALL(Gondo_GPS, Vane_GPS, Sail_GPS, Ground_GPS, Elapsed_1, Elapsed_2)
% PlotGPSTestALL(Gondo_GPS, Vane_GPS, Sail_GPS, Ground_GPS, Elapsed_1, Elapsed_2) 
% takes the data in Gondo, Vane, Sail, and Ground station and returns
% figures of the data. 
%
% INPUTS
% Gondo_GPS     = Gondola GPS structure
% Vane_GPS      = Vane GPS structure (can replace with OG ground station)
% Sail_GPS      = Sail GPS structure
% Ground_GPS    = Ground GPS structure (new ground station)
% Elapsed_1     = starting times for each new trial in elapsed seconds
% Elapsed_2     = starting times for each new trial in elapsed seconds

% input handling 
trialflag = 0;
if isempty(Elapsed_1) + isempty(Elapsed_2) == 2
    trialflag = 1;
end

% pull data for each test
GGA_Gondo_lat = Gondo_GPS.GPGGA(:, 4);  % lat
GGA_Gondo_lon = Gondo_GPS.GPGGA(:, 5);  % lon
GGA_Gondo_alt = Gondo_GPS.GPGGA(:, 9);  % alt
GGA_Gondo_tme = Gondo_GPS.GPGGA(:, 11); % elapsed time
GGA_Vane_lat = Vane_GPS.GPGGA(:, 4);  % lat
GGA_Vane_lon = Vane_GPS.GPGGA(:, 5);  % lon
GGA_Vane_alt = Vane_GPS.GPGGA(:, 9);  % alt
GGA_Vane_tme = Vane_GPS.GPGGA(:, 11); % elapsed time
GGA_Sail_lat = Sail_GPS.GPGGA(:, 4);  % lat
GGA_Sail_lon = Sail_GPS.GPGGA(:, 5);  % lon
GGA_Sail_alt = Sail_GPS.GPGGA(:, 9);  % alt
GGA_Sail_tme = Sail_GPS.GPGGA(:, 11); % elapsed time
GGA_Ground_lat = Ground_GPS.GPGGA(:, 4);  % lat
GGA_Ground_lon = Ground_GPS.GPGGA(:, 5);  % lon
GGA_Ground_alt = Ground_GPS.GPGGA(:, 9);  % alt
GGA_Ground_tme = Ground_GPS.GPGGA(:, 11); % elapsed time


figure('Color', 'w');
subplot(3, 1, 1);
hold on
% % elapsed time
% plot(GGA_Gondo_tme - GGA_Gondo_tme(1), GGA_Gondo_lat);
% plot(GGA_Vane_tme - GGA_Vane_tme(1), GGA_Vane_lat);
% plot(GGA_Sail_tme - GGA_Sail_tme(1), GGA_Sail_lat);
% time since midnight
p1 = plot(GGA_Gondo_tme, GGA_Gondo_lat);
p2 = plot(GGA_Vane_tme, GGA_Vane_lat);
p3 = plot(GGA_Sail_tme, GGA_Sail_lat);
p4 = plot(GGA_Ground_tme, GGA_Ground_lat);
ylabel('Latitude [deg]');
grid on
if trialflag == 0
    ylimz = ylim;
    for i1 = 1:length(Elapsed_1)
        plot(Elapsed_1(i1)*[1, 1], ylimz, 'r--');
    end
    for i1 = 1:length(Elapsed_2)
        plot(Elapsed_2(i1)*[1, 1], ylimz, 'b--');
    end
end
legend([p1, p2, p3, p4], {'Gondo', 'Vane', 'Sail', 'Ground'}, 'location', 'northwest');
subplot(3, 1, 2);
hold on
% % elapsed time
% plot(GGA_Gondo_tme - GGA_Gondo_tme(1), GGA_Gondo_lon);
% plot(GGA_Vane_tme - GGA_Vane_tme(1), GGA_Vane_lon);
% plot(GGA_Sail_tme - GGA_Sail_tme(1), GGA_Sail_lon);
% time since midnight
plot(GGA_Gondo_tme, GGA_Gondo_lon);
plot(GGA_Vane_tme, GGA_Vane_lon);
plot(GGA_Sail_tme, GGA_Sail_lon);
plot(GGA_Ground_tme, GGA_Ground_lon);
ylabel('Longitude [deg]');
grid on
if trialflag == 0
    ylim(ylimz);
    for i1 = 1:length(Elapsed_1)
        plot(Elapsed_1(i1)*[1, 1], ylimz, 'r--');
    end
    for i1 = 1:length(Elapsed_2)
        plot(Elapsed_2(i1)*[1, 1], ylimz, 'b--');
    end
end
subplot(3, 1, 3);
hold on
% % elapsed time
% plot(GGA_Gondo_tme - GGA_Gondo_tme(1), GGA_Gondo_alt);
% plot(GGA_Vane_tme - GGA_Vane_tme(1), GGA_Vane_alt);
% plot(GGA_Sail_tme - GGA_Sail_tme(1), GGA_Sail_alt);

% time since midnight
plot(GGA_Gondo_tme, GGA_Gondo_alt);
plot(GGA_Vane_tme, GGA_Vane_alt);
plot(GGA_Sail_tme, GGA_Sail_alt);
plot(GGA_Ground_tme, GGA_Ground_alt);
ylabel('Altitude [m]');
xlabel('Time [s]');
grid on
if trialflag == 0
    ylimz = ylim;
    for i1 = 1:length(Elapsed_1)
        plot(Elapsed_1(i1)*[1, 1], ylimz, 'r--');
        text(Elapsed_1(i1) + 20, 165, ['Test1 #', num2str(i1), ' \downarrow'], ...
            'Color', 'r', 'Rotation', 90, 'FontName', 'TimesNewRoman', 'FontSize', 6);
    end
    for i1 = 1:length(Elapsed_2)
        plot(Elapsed_2(i1)*[1, 1], ylimz, 'b--');
        text(Elapsed_2(i1) + 20, 165, ['Test2 #', num2str(i1), ' \downarrow'], ...
            'Color', 'b', 'Rotation', 90, 'FontName', 'TimesNewRoman', 'FontSize', 6);
    end
end 

end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function [plotData] = PlotAOData(Gondo_AO, Vane_AO, NewSail_AO, Ground_AO, varargin)
% Returns a plot of the specified data from the gondola, vane, sail, and
% ground station. Varargin can be any of the following name/value pairs:
%   'timebounds' | [start, end] = used to plot data for a given interval of
%   start and end. The value is a two element vector of start and stop
%   times. 
%   'data' | 'accl', 'mag', 'angvel', 'grav', 'wind', 'euler', 'head'
%   Used to specify what data to plot. Default is acceleration. 
%   'rho' | 1.225,  density of the atmosphere at ground level
%   'magdec' | 0 deg, magnetic declination angle for flight location. 
%   '
%   
% INPUTS:
%   Gondo_AO    = gondola AO data matrix
%   Vane_AO     = Vane AO data matrix
%   NewSail_AO  = NewSail AO data matrix
%   Ground_AO   = Ground AO data matrix
% 
% References:
% https://aerocontent.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf


% break out
Gondo_tme = Gondo_AO(:, end);
Gondo_accl = Gondo_AO(:, 6:8);
Gondo_angvel = Gondo_AO(:, 9:11);
Gondo_mag = Gondo_AO(:, 12:14);
Gondo_euler = Gondo_AO(:, 15:17);
Gondo_grav = Gondo_AO(:, 18:20);
Gondo_wind = Gondo_AO(:, 25);
Ts1 = 1;                    % index of start time
Te1 = length(Gondo_tme);    % index of end time

Vane_tme = Vane_AO(:, end);
Vane_accl = Vane_AO(:, 6:8);
Vane_angvel = Vane_AO(:, 9:11);
Vane_mag = Vane_AO(:, 12:14);
Vane_euler = Vane_AO(:, 15:17);
Vane_grav = Vane_AO(:, 18:20);
Vane_wind = Vane_AO(:, 25);
Ts2 = 1;
Te2 = length(Vane_tme);

Sail_tme = NewSail_AO(:, end);
Sail_accl = NewSail_AO(:, 6:8);
Sail_angvel = NewSail_AO(:, 9:11);
Sail_mag = NewSail_AO(:, 12:14);
Sail_euler = NewSail_AO(:, 15:17);
Sail_grav = NewSail_AO(:, 18:20);
Sail_wind = NewSail_AO(:, 25);
Ts3 = 1;
Te3 = length(Sail_tme);

%   [hr, mn, sc, ms, alt, ax, ay, az, wx, wy, wz, mx, my, mz, ...
%       ex, ey, ez, gx, gy, gz, temp, speed, gust, dirn, elpsdtme]
Ground_tme = Ground_AO(:, end);
Ground_accl = Ground_AO(:, 6:8);
Ground_angvel = Ground_AO(:, 9:11);
Ground_mag = Ground_AO(:, 12:14);
Ground_euler = Ground_AO(:, 15:17);
Ground_grav = Ground_AO(:, 18:20);
Ground_temp = Ground_AO(:, 21);
Ground_wind = Ground_AO(:, 22);
Ground_gust = Ground_AO(:, 23);
Ground_dirn = Ground_AO(:, 24);
Ground_alt = Ground_AO(:, 5);
Ts4 = 1;
Te4 = length(Ground_tme);

% handle plot stuffs
xlabels = 'Elapsed time [s]';
plotData.Gondo = Gondo_accl;
plotData.Vane = Vane_accl;
plotData.Sail = Sail_accl;
plotData.Ground = Ground_accl;
ylabels = 'Acceleration [m/s2]';
indxFlag = 3;
lgnd = {'X', 'Y', 'Z'};
calflag = 0;    % no calibration, report bins
rho = 1.225;    % kg.m3
mag_dec = 0;    % magnetic declination angle
dirFlag = 0;    % correct direction readings on gnd station
ttlstr = ' ';
pitot1 = [0, 0, 0];
pitot2 = [0, 0, 0];
pitot3 = [0, 0, 0];
pitot4 = [0, 0, 0];
bndsFlag = NaN(1, 4);
figFlag = 0;
allFlag = 0;
if isempty(varargin) ~= 1
    for i1 = 1:length(varargin)/2
        arg2 = varargin{2*(i1 - 1) + 2};
        switch varargin{2*(i1 - 1) + 1}
            case 'data'
                switch arg2
                    case 'accl'
                        plotData.Gondo = Gondo_accl;
                        plotData.Vane = Vane_accl;
                        plotData.Sail = Sail_accl;
                        plotData.Ground = Ground_accl;
                        ylabels = 'Acceleration [m/s2]';
                        indxFlag = 3;
                        lgnd = {'X', 'Y', 'Z'};
                    case 'angvel'
                        plotData.Gondo = Gondo_angvel;
                        plotData.Vane = Vane_angvel;
                        plotData.Sail = Sail_angvel;
                        plotData.Ground = Ground_angvel;
                        ylabels = 'Angular velocity [deg/s]';
                        indxFlag = 3;
                        lgnd = {'X', 'Y', 'Z'};
                    case 'mag'
                        plotData.Gondo = Gondo_mag;
                        plotData.Vane = Vane_mag;
                        plotData.Sail = Sail_mag;
                        plotData.Ground = Ground_mag;
                        ylabels = 'Magnetic lines [uT]';
                        indxFlag = 3;
                        lgnd = {'X', 'Y', 'Z'};
                    case 'grav'
                        plotData.Gondo = Gondo_grav;
                        plotData.Vane = Vane_grav;
                        plotData.Sail = Sail_grav;
                        plotData.Ground = Ground_grav;
                        ylabels = 'Gravity [m/s2]';
                        indxFlag = 3;
                        lgnd = {'X', 'Y', 'Z'};
                    case 'wind'
                        plotData.Gondo = Gondo_wind;
                        plotData.Vane = Vane_wind;
                        plotData.Sail = Sail_wind;
                        plotData.Ground = Ground_wind;
                        ylabels = 'Wind speed [m/s]';
                        indxFlag = 1;
                        lgnd = {'Wind Speed'};
                    case 'euler'
                        for i2 = 1:3
                            angout1 = WholeToHalf(Gondo_euler(:, i2));
                            plotData.Gondo(:, i2) = angout1;
                            angout2 = WholeToHalf(Vane_euler(:, i2));
                            plotData.Vane(:, i2) = angout2;
                            angout3 = WholeToHalf(Sail_euler(:, i2));
                            plotData.Sail(:, i2) = angout3;
                            angout4 = WholeToHalf(Ground_euler(:, i2));
                            plotData.Ground(:, i2) = angout4;
                        end
                        ylabels = 'Euler angles [deg]';
                        indxFlag = 3;
                        lgnd = {'X', 'Y', 'Z'};
                    case 'head'
                        d1 = GetDirection(Gondo_mag(:, 1), Gondo_mag(:, 2));
                        plotData.Gondo = d1;
                        d1 = GetDirection(Vane_mag(:, 1), Vane_mag(:, 2));
                        plotData.Vane = d1;
                        d1 = GetDirection(Sail_mag(:, 1), Sail_mag(:, 2));
                        plotData.Sail = d1;
                        d1 = GetDirection(Ground_mag(:, 1), Ground_mag(:, 2));
                        plotData.Ground = d1;
                        indxFlag = 1;
                        lgnd = {'Heading'};
                        ylabels = 'Heading [deg]';
                    case 'all'
                        allFlag = 1;
                        figFlag = -1;
                    otherwise
                        error('Second argument not recognized.');
                end
            case 'timebounds'
                Ts = arg2(1);
                Te = arg2(2);
                Ts1 = sum(Gondo_tme <= Ts);
                Te1 = sum(Gondo_tme < Te);
                if Ts1 == Te1
                    Ts1 = 1;
                    Te1 = length(Gondo_tme);
                    bndsFlag(1) = 1;
                end
                Ts2 = sum(Vane_tme <= Ts);
                Te2 = sum(Vane_tme < Te);
                if Ts2 == Te2
                    Ts2 = 1;
                    Te2 = length(Vane_tme);
                    bndsFlag(2) = 1;
                end
                Ts3 = sum(Sail_tme <= Ts);
                Te3 = sum(Sail_tme < Te);
                if Ts3 == Te3
                    Ts3 = 1;
                    Te3 = length(Sail_tme);
                    bndsFlag(3) = 1;
                end
                Ts4 = sum(Ground_tme <= Ts);
                Te4 = sum(Ground_tme < Te);
                if Ts4 == Te4
                    Ts4 = 1;
                    Te4 = length(Ground_tme);
                    bndsFlag(4) = 1;
                end
                
            
            case 'pitotcal'
                switch arg2
                    case 'bins'
                        calflag = 0;
                        
                    case 'generic'
                        % Use the generic calibration for pressure sensor
                        % Sensor is Pressure = +/- 2kPa
                        % Density is 1.225 kg/m3
                        % ADC is 10 bit (1024 bin)
                        % voltage is 0-5 VDC, 2.5V = 0 Pa
                        % Velociyt = SQRT((2*(F8*(5/1024)/0.00125-2000))/1.225)
                        calflag = 1;
                        
                    case '1-9-19-Cal'
                        % Uses Saurabh calibration from 1-9-19
                        pitot1 = [0.027661, 0.639006, 536.127451];
                        pitot2 = [0.109944, -0.157563, 542.274510];
                        pitot3 = [0.079132, 0.016106, 539.921569];
                        pitot4 = [0.038165, 0.390406, 541.323529];
                        calflag = 2;                        
                        
                    case '8-10-19-Cal'
                        % Use Saruabh's wind tunnel calibration for the
                        % 8-10-19 test
                        % G bin1 = 361.6 * spd + 8702
                        % R bin2 = 355.5 * spd + 8662
                        % S bin3 = 324.6 * spd + 8752
                        pitot1 = [1, -8702]/361.6;
                        pitot2 = [1, -8662]/355.5;
                        pitot3 = [1, -8752]/324.6;
                        calflag = 3;
                        
                    otherwise
                        error('Pitot calibration argument is not recognized.');
                end
                
            case 'rho'
                rho = arg2; % enter new value for rho
                
            case 'magdec'
                mag_dec = arg2;
                
            case 'dircal'
                if strcmp(arg2, 'solve') == 1
                    dirFlag = 1;
                    dircal = 270 - median(Ground_dirn);
                elseif isnumeric(arg2) == 1
                    dirFlag = 1;
                    dircal = arg2;
                else
                    error('dircal value not recognized.');
                end
                
            case 'title'
                ttlstr = arg2;
                
            case 'plots'
                if strcmp(arg2, 'none') == 1
                    figFlag = -1;
                elseif strcmp(arg2, 'all') == 1
                    figFlag = 0;
                else
                    error('Figure flag argument not recognized.');
                end
                
            otherwise
                error('First argument not recognized.');
        end
    end
end

% handle pitot calibration as needed
% plotData.Gondo = Gondo_wind;
% plotData.Vane = Vane_wind;
% plotData.Sail = Sail_wind;
% plotData.Ground = Ground_wind;
% ylabels = 'Wind speed [m/s]';
% indxFlag = 1;
% lgnd = {'Wind Speed'};
switch calflag
    case 0
    case 1
        % use generic calibration
        winds = plotData.Gondo;
        plotData.Gondo = sqrt((2*(winds*(5./1024)./0.00125-2000))./rho);    % convert bins to m/s
        winds = plotData.Vane;
        plotData.Vane = sqrt((2*(winds*(5./1024)./0.00125-2000))./rho);    % convert bins to m/s
        winds = plotData.Sail;
        plotData.Sail = sqrt((2*(winds*(5./1024)./0.00125-2000))./rho);    % convert bins to m/s
    case 2
        % use 1-9-19 calibration
        % assume 1 = sail, 2 = vane, 3 = gondo, 4 = spare
        winds3 = plotData.Gondo;
%         newgond = NaN*winds3;
%         for k1 = 1:length(winds3(:, 1))
%             newgond(k1) = max(roots(pitot3 - [0, 0, winds3(k1)]));
%         end
%         plotData.Gondo = newgond;
        bin0 = 758; % min bin value of gondo data
        plotData.Gondo = sqrt((2*((winds3 - bin0 + 512)*(5./1024)./0.00125-2000))./rho);
        
        winds2 = plotData.Vane;
        newvane = NaN*winds2;
        for k1 = 1:length(winds2(:, 1))
            newvane(k1) = max(roots(pitot2 - [0, 0, winds2(k1)]));
        end
        plotData.Vane = newvane;
        
        winds1 = plotData.Sail;
        newsail = NaN*winds1;
        for k1 = 1:length(winds1(:, 1))
            newsail(k1) = max(roots(pitot1 - [0, 0, winds1(k1)]));
        end
        plotData.Sail = newsail;          
            
    case 3
        % use cal data from 8-10-2019 test
        % assume pitot1 = sail, pitot2 = n/a, pitot3 = gondo
        winds1 = plotData.Sail; % bins
        plotData.Sail = pitot1(1).*winds1 + pitot1(2);
        winds1 = plotData.Gondo; % bins
        plotData.Gondo = pitot3(1).*winds1 + pitot3(2);
        
    otherwise
end

% handle magnetic declination angle correction
if mag_dec ~= 0
    plotData.Gondo = plotData.Gondo + mag_dec;
    plotData.Vane = plotData.Vane + mag_dec;
    plotData.Sail = plotData.Sail + mag_dec;
    plotData.Ground = plotData.Ground + mag_dec;
end

% handle direction offset for GND station
if dirFlag == 1
    plotData.Ground = plotData.Ground + dircal;
end

% if all flag
if allFlag == 1
    
    % save outputs
    plotData.Gondo1.Gondo_tme = Gondo_tme(Ts1:Te1, :);
    plotData.Gondo1.Gondo_accl = Gondo_accl(Ts1:Te1, :);
    plotData.Gondo1.Gondo_angvel = Gondo_angvel(Ts1:Te1, :);
    plotData.Gondo1.Gondo_mag = Gondo_mag(Ts1:Te1, :);
    plotData.Gondo1.Gondo_euler = Gondo_euler(Ts1:Te1, :);
    plotData.Gondo1.Gondo_grav = Gondo_grav(Ts1:Te1, :);
    plotData.Gondo1.Gondo_wind = Gondo_wind(Ts1:Te1, :);
    
    plotData.Vane1.Vane_tme = Vane_tme(Ts1:Te1, :);
    plotData.Vane1.Vane_accl = Vane_accl(Ts1:Te1, :);
    plotData.Vane1.Vane_angvel = Vane_angvel(Ts1:Te1, :);
    plotData.Vane1.Vane_mag = Vane_mag(Ts1:Te1, :);
    plotData.Vane1.Vane_euler = Vane_euler(Ts1:Te1, :);
    plotData.Vane1.Vane_grav = Vane_grav(Ts1:Te1, :);
    plotData.Vane1.Vane_wind = Vane_wind(Ts1:Te1, :);
    
    plotData.Sail1.Sail_tme = Sail_tme(Ts1:Te1, :);
    plotData.Sail1.Sail_accl = Sail_accl(Ts1:Te1, :);
    plotData.Sail1.Sail_angvel = Sail_angvel(Ts1:Te1, :);
    plotData.Sail1.Sail_mag = Sail_mag(Ts1:Te1, :);
    plotData.Sail1.Sail_euler = Sail_euler(Ts1:Te1, :);
    plotData.Sail1.Sail_grav = Sail_grav(Ts1:Te1, :);
    plotData.Sail1.Sail_wind = Sail_wind(Ts1:Te1, :);
    
    plotData.Ground1.Ground_tme = Ground_tme(Ts1:Te1, :);
    plotData.Ground1.Ground_accl = Ground_accl(Ts1:Te1, :);
    plotData.Ground1.Ground_angvel = Ground_angvel(Ts1:Te1, :);
    plotData.Ground1.Ground_mag = Ground_mag(Ts1:Te1, :);
    plotData.Ground1.Ground_euler = Ground_euler(Ts1:Te1, :);
    plotData.Ground1.Ground_grav = Ground_grav(Ts1:Te1, :);
    plotData.Ground1.Ground_temp = Ground_temp(Ts1:Te1, :);
    plotData.Ground1.Ground_wind = Ground_wind(Ts1:Te1, :);
    plotData.Ground1.Ground_gust = Ground_gust(Ts1:Te1, :);
    plotData.Ground1.Ground_dirn = Ground_dirn(Ts1:Te1, :);
    plotData.Ground1.Ground_alt = Ground_alt(Ts1:Te1, :);
    
end

% plotting
if figFlag ~= -1
    v1 = [];
    v2 = [];
    ttl = 4;
    figure('Color', 'w');
    hold on
    subplot(ttl, 1, 1)
    hold on
    for i1 = 1:indxFlag
        plot(Gondo_tme(Ts1:Te1), plotData.Gondo(Ts1:Te1, i1));
    end
    % xlabel(xlabels);
    ylabel({'Gondola', ylabels});
    title('Gondola');
    grid on
    legend(lgnd, 'location', 'best');
    if isnan(bndsFlag(1)) == 0
        xlimz1 = xlim;
        v1 = [v1, xlimz1(1)];
        v2 = [v2, xlimz1(2)];
    end
    title(ttlstr);

    subplot(ttl, 1, 2)
    hold on
    for i1 = 1:indxFlag
        plot(Vane_tme(Ts2:Te2), plotData.Vane(Ts2:Te2, i1));
    end
    % xlabel(xlabels);
    ylabel({'Vane', ylabels});
    % title('Vane');
    grid on
    if isnan(bndsFlag(2)) == 0
        xlimz2 = xlim;
        v1 = [v1, xlimz2(1)];
        v2 = [v2, xlimz2(2)];
    end

    subplot(ttl, 1, 3)
    hold on
    for i1 = 1:indxFlag
        plot(Sail_tme(Ts3:Te3), plotData.Sail(Ts3:Te3, i1));
    end
    % title('Sail');
    % xlabel(xlabels);
    ylabel({'Sail', ylabels});
    grid on
    if isnan(bndsFlag(3)) == 0
        xlimz3 = xlim;
        v1 = [v1, xlimz3(1)];
        v2 = [v2, xlimz3(2)];
    end

    subplot(ttl, 1, 4)
    hold on
    for i1 = 1:indxFlag
        plot(Ground_tme(Ts4:Te4), plotData.Ground(Ts4:Te4, i1));
    end
    % title('Ground station');
    xlabel(xlabels);
    ylabel({'Ground station', ylabels});
    grid on
    if isnan(bndsFlag(4)) == 0
        xlimz4 = xlim;
        v1 = [v1, xlimz4(1)];
        v2 = [v2, xlimz4(2)];
    end

    % reset axis limits to make them all the same
    % xlimMIN = min([xlimz1(1), xlimz2(1), xlimz3(1), xlimz4(1)]);
    % xlimMAX = min([xlimz1(2), xlimz2(2), xlimz3(2), xlimz4(2)]);
    if isempty(v1) ~= 1
        xlimMIN = max(v1);
        xlimMAX = min(v2);
        for i1 = 1:4
            subplot(4, 1, i1);
            set(gca, 'xlim', [xlimMIN, xlimMAX]);
        end
    end
end

% add other things to plotdata
plotData.TsGondo = Ts1;
plotData.TeGondo = Te1;
plotData.TsVane = Ts2;
plotData.TeVane = Te2;
plotData.TsSail = Ts3;
plotData.TeSail = Te3;
plotData.TsGround = Ts4;
plotData.TeGround = Te4;
plotData.Gondo_tme = Gondo_tme;
plotData.Vane_tme = Vane_tme;
plotData.Sail_tme = Sail_tme;
plotData.Ground_tme = Ground_tme;

end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function [lat0, lon0, alt0, newMove, errs, stats] = dGPS(Station, Mover, varargin)
% [lat0, lon0, alt0, errs] = dGPS(Station, mover, varargin) returns
% the nominal lat0, lon0, and alt0 of a stationary ground station in
% addition to the time errors of a moving GPS point (mover). 
% 
% INPUT:
% Station   = a matrix of [time, lat, lon, alt]
% Mover     = a matrix of [time, lat, lon, alt]
% 
% OUTPUT:
% lat0      = time average latitude of station
% lon0      = time average longitude of station
% alt0      = time average altitude of station
% errs      = array of [time, lat_err, lon_err, alt_err] in the Mover times
% newMove   = Mover data corrected for the errors
%
% References:
% https://racelogic.support/01VBOX_Automotive/01General_Information/Knowledge_Base/How_does_DGPS_(Differential_GPS)_work%3F
% https://www.e-education.psu.edu/geog160/node/1925

% errors
if length(Station(1, :)) ~= 4
    error('Station does not have four columns.');
end
if length(Mover(1, :)) ~= 4
    error('Mover does not have four columns.');
end

% get averages
lat0 = mean(Station(:, 2));
lon0 = mean(Station(:, 3));
alt0 = mean(Station(:, 4));
stats.lat_avg = lat0;
stats.lon_avg = lon0;
stats.alt_avg = alt0;
stats.lat_std = std(Station(:, 2));
stats.lon_std = std(Station(:, 3));
stats.alt_std = std(Station(:, 4));
stats.lat_med = median(Station(:, 2));
stats.lon_med = median(Station(:, 3));
stats.alt_med = median(Station(:, 4));

% get errors in station time
Station_errs = NaN*Station;
Station_errs(:, 1) = Station(:, 1);         % times
Station_errs(:, 2) = Station(:, 2) - lat0;  % Actual - avg
Station_errs(:, 3) = Station(:, 3) - lon0;  % Actual - avg
Station_errs(:, 4) = Station(:, 4) - alt0;  % Actual - avg

% correct for errors in movers
errs = NaN*Mover;
errs(:, 1) = Mover(:, 1);
errs(:, 2) = interp1(Station_errs(:, 1), Station_errs(:, 2), Mover(:, 1), 'linear', 'extrap');  % mover lat err
errs(:, 3) = interp1(Station_errs(:, 1), Station_errs(:, 3), Mover(:, 1), 'linear', 'extrap');  % mover lon err
errs(:, 4) = interp1(Station_errs(:, 1), Station_errs(:, 4), Mover(:, 1), 'linear', 'extrap');  % mover alt err

% fix data
newMove = NaN*errs;
newMove(:, 1) = Mover(:, 1);
newMove(:, 2) = Mover(:, 2) - errs(:, 2);   % corrected latitude of mover
newMove(:, 3) = Mover(:, 3) - errs(:, 3);   % corrected longitude of mover
newMove(:, 4) = Mover(:, 4) - errs(:, 4);   % corrected altitude of mover

% handle varargin if present
plotflag = 0;
if isempty(varargin) ~= 1
    n = length(varargin);
    for i1 = 1:n/2
        arg1 = varargin{2*(i1 - 1) + 1};
        arg2 = varargin{2*(i1 - 1) + 2};
        switch arg1
            case 'plot'
                if strcmp(arg2, 'off') == 1
                    plotflag = 0;
                elseif strcmp(arg2, 'station') == 1
                    plotflag = 1;
                else
                    error('Plot is not either off or on.');
                end
        end
    end
end

% handle plotting
if plotflag == 1
    figure('Color', 'w');
    hold on
    plot(Station(:, 3), Station(:, 2), 'o');
    plot(lon0, lat0, 'r+', 'MarkerSize', 12);
    xlabel('Longitude (deg)');
    ylabel('Latitude (deg)');
    grid on
    keyboard
    close(gcf);
end

end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function [dirctn] = GetDirection(Hx, Hy)
% Function to calculate the heading from magnetic lines

dirctn = NaN*Hx;
for i1 = 1:length(Hx)

    if Hy(i1) > 0
        dirctn(i1) = 90 - (180/pi)*atan(Hx(i1)/Hy(i1));
    elseif Hy(i1) < 0
        dirctn(i1) = 270 - (180/pi)*atan(Hx(i1)/Hy(i1));
    elseif Hy(i1) == 0 && Hx(i1) < 0
        dirctn(i1) = 180;
    elseif Hy(i1) == 0 && Hx(i1) > 0 
        dirctn(i1) = 0;
    elseif isnan(Hy(i1)) == 1 || isnan(Hx(i1)) == 1
        dirctn(i1) = NaN;
    else
        keyboard
        error('None of the criteria met.');
    end
    
end

end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function [dec_deg] = DMStoDD(deg, min, sec, sgn)
% Returns the signed degrees of an angle given in decrees minutes seconds. 
% 
% INPUT
%   deg     = degrees
%   min     = minutes
%   sec     = seconds
%   sng     = direction of angle, either 'N', 'S', 'E', 'W'. For no direciton ,
%             use 'N'.
% OUTPUT
%   dec_deg = decimal degrees, signed
%
% Reference:
% https://www.rapidtables.com/convert/number/degrees-minutes-seconds-to-degrees.html

% make it a loop for everybody
dec_deg = NaN*deg;
for i1 = 1:length(deg)

    % check stuffs
    sgn1 = 1;
    switch sgn(i1)
        case 'N'
            sgn1 = 1;
        case 'S'
            sgn1 = -1;
        case 'E'
            sgn1 = 1;
        case 'W'
            sgn1 = -1;
        otherwise
            error('sgn argument not recognized.');
    end

    % do math
    dec_deg(i1) = sgn1*(deg(i1) + min(i1)/60 + sec(i1)/3600);

end

end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function [datastats] = CalcStats(datavec)
% [datastats] = CalcStats(datavec) returns the following for the
% datavec:
%   datastats.xbar = average
%   datastats.sigm = standard deviation
%   datastats.numb = number of data points
%   datastats.medi = median

% get stuffs
datastats.xbar = mean(datavec);
datastats.sigm = std(datavec);
datastats.numb = length(datavec);
datastats.medi = median(datavec);

end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function [windspeed] = ConvertToWindSpeed(binvec, bin0, rho0)
% converts bin values to wind speed with a DC offset of bin0 and density of
% rho0. 

windspeed = sqrt((2*((binvec - bin0 + 512)*(5./1024)./0.00125-2000))./rho0);

end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function [angout] = WholeToHalf(angin)
% Convert angin from [0, 360] to [-180, 180].

% errors
if isnumeric(angin) ~=1
    error('angin must be numeric.');
end

% convert
angtemp = (angin>180)*-360 + angin;
angout = (angtemp<-180)*360 + angtemp;

end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function WriteToTextFile(dataset, filetype, filename)
%
% dataset needs to be of the form (gondola, vane, sail)
% = [time, lat, lon, alt, X, Y, Z, ax, ay, az, wx, wy, wz, mx, my, mz, ...
%       ex, ey, ez, gx, gy, gz, temp, pres, humd, windspd]
% 
% dataset needs to be of the form (gondola, vane, sail)
% = [time, lat, lon, alt, X, Y, Z, ax, ay, az, wx, wy, wz, mx, my, mz, ...
%       ex, ey, ez, gx, gy, gz, temp, speed, gust, dir]

% write the file
headrow = 'time (s), lat (deg), lon (deg), alt (m), X (m), Y (m), Z (m), ax (m/s2), ay (m/s2), az (m/s2), wx (deg/s), wy (deg/s), wz (deg/s), mx (uT), my (uT), mz (uT), ex (deg), ey (deg), ez (deg), gx (m/s2), gy (m/s2), gz (m/s2), WindSpeed (bin)';
switch filetype
    case 'ground'
        headrow = 'time (s), lat (deg), lon (deg), alt (m), X (m), Y (m), Z (m), ax (m/s2), ay (m/s2), az (m/s2), wx (deg/s), wy (deg/s), wz (deg/s), mx (uT), my (uT), mz (uT), ex (deg), ey (deg), ez (deg), gx (m/s2), gy (m/s2), gz (m/s2), T (degC), WindSpeed (m/s), WindGust (m/s), WindDir (deg)';
    otherwise
end
fid = fopen(filename, 'w');
fprintf(fid, '%s\n', headrow);
fclose(fid);
dlmwrite(filename, dataset, '-append');

end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function [outvec] = MakeMatrix(XYZ, AO, filetype)

% all XYZ data
% testdata.Ground = [Ground_tme, Ground_x, Ground_y, Ground_z];
% testdata.GroundLLA = [Ground_tme, Ground_lat, Ground_lon, Ground_alt];

% gondola, sail, vane - AO
% plotData.Gondo.Gondo_tme = Gondo_tme(Ts1:Te1, :);
% plotData.Gondo.Gondo_accl = Gondo_accl(Ts1:Te1, :);
% plotData.Gondo.Gondo_angvel = Gondo_angvel(Ts1:Te1, :);
% plotData.Gondo.Gondo_mag = Gondo_mag(Ts1:Te1, :);
% plotData.Gondo.Gondo_euler = Gondo_euler(Ts1:Te1, :);
% plotData.Gondo.Gondo_grav = Gondo_grav(Ts1:Te1, :);
% plotData.Gondo.Gondo_wind = Gondo_wind(Ts1:Te1, :);

switch filetype
    case 'ground'
        % dataset needs to be of the form (gondola, vane, sail)
        % = [time, lat, lon, alt, X, Y, Z, ax, ay, az, wx, wy, wz, mx, my, mz, ...
        %       ex, ey, ez, gx, gy, gz, temp, speed, gust, dir]
        lngt = min([length(XYZ.GroundLLA(:, 1)), length(AO.Ground1.Ground_accl(:, 1)), ...
            length(AO.Ground1.Ground_angvel(:, 1)), length(AO.Ground1.Ground_mag(:, 1)), ...
            length(AO.Ground1.Ground_euler(:, 1)), length(AO.Ground1.Ground_grav(:, 1)), ...
            length(AO.Ground1.Ground_temp(:, 1)), length(AO.Ground1.Ground_wind(:, 1)), ...
            length(AO.Ground1.Ground_gust(:, 1)), length(AO.Ground1.Ground_dirn(:, 1))]);
            
        outvec = [XYZ.GroundLLA(1:lngt, :), XYZ.Ground(1:lngt, 2:4), AO.Ground1.Ground_accl(1:lngt, :), ...
            AO.Ground1.Ground_angvel(1:lngt, :), AO.Ground1.Ground_mag(1:lngt, :), AO.Ground1.Ground_euler(1:lngt, :), ...
            AO.Ground1.Ground_grav(1:lngt, :), AO.Ground1.Ground_temp(1:lngt, :), AO.Ground1.Ground_wind(1:lngt, :), ...
            AO.Ground1.Ground_gust(1:lngt, :), AO.Ground1.Ground_dirn(1:lngt, :)];
        
    case 'gondo'
        % dataset needs to be of the form (gondola, vane, sail)
        % = [time, lat, lon, alt, X, Y, Z, ax, ay, az, wx, wy, wz, mx, my, mz, ...
        %       ex, ey, ez, gx, gy, gz, windspd]
        lngt = min([length(XYZ.GondoLLA(:, 1)), length(AO.Gondo1.Gondo_accl(:, 1)), ...
            length(AO.Gondo1.Gondo_angvel(:, 1)), length(AO.Gondo1.Gondo_mag(:, 1)), ...
            length(AO.Gondo1.Gondo_euler(:, 1)), length(AO.Gondo1.Gondo_grav(:, 1)), ...
            length(AO.Gondo1.Gondo_wind(:, 1))]);
        
        outvec = [XYZ.GondoLLA(1:lngt, :), XYZ.Gondo(1:lngt, 2:4), AO.Gondo1.Gondo_accl(1:lngt, :), ...
            AO.Gondo1.Gondo_angvel(1:lngt, :), AO.Gondo1.Gondo_mag(1:lngt, :), AO.Gondo1.Gondo_euler(1:lngt, :), ...
            AO.Gondo1.Gondo_grav(1:lngt, :), AO.Gondo1.Gondo_wind(1:lngt, :)];
        
    case 'vane'
        % dataset needs to be of the form (gondola, vane, sail)
        % = [time, lat, lon, alt, X, Y, Z, ax, ay, az, wx, wy, wz, mx, my, mz, ...
        %       ex, ey, ez, gx, gy, gz, windspd]
        lngt = min([length(XYZ.VaneLLA(:, 1)), length(AO.Vane1.Vane_accl(:, 1)), ...
            length(AO.Vane1.Vane_angvel(:, 1)), length(AO.Vane1.Vane_mag(:, 1)), ...
            length(AO.Vane1.Vane_euler(:, 1)), length(AO.Vane1.Vane_grav(:, 1)), ...
            length(AO.Vane1.Vane_wind(:, 1))]);
        
        outvec = [XYZ.VaneLLA(1:lngt, :), XYZ.Vane(1:lngt, 2:4), AO.Vane1.Vane_accl(1:lngt, :), ...
            AO.Vane1.Vane_angvel(1:lngt, :), AO.Vane1.Vane_mag(1:lngt, :), AO.Vane1.Vane_euler(1:lngt, :), ...
            AO.Vane1.Vane_grav(1:lngt, :), AO.Vane1.Vane_wind(1:lngt, :)];
        
    case 'sail'
        % dataset needs to be of the form (gondola, vane, sail)
        % = [time, lat, lon, alt, X, Y, Z, ax, ay, az, wx, wy, wz, mx, my, mz, ...
        %       ex, ey, ez, gx, gy, gz, windspd]
        lngt = min([length(XYZ.SailLLA(:, 1)), length(AO.Sail1.Sail_accl(:, 1)), ...
            length(AO.Sail1.Sail_angvel(:, 1)), length(AO.Sail1.Sail_mag(:, 1)), ...
            length(AO.Sail1.Sail_euler(:, 1)), length(AO.Sail1.Sail_grav(:, 1)), ...
            length(AO.Sail1.Sail_wind(:, 1))]);
        
        outvec = [XYZ.SailLLA(1:lngt, :), XYZ.Sail(1:lngt, 2:4), AO.Sail1.Sail_accl(1:lngt, :), ...
            AO.Sail1.Sail_angvel(1:lngt, :), AO.Sail1.Sail_mag(1:lngt, :), AO.Sail1.Sail_euler(1:lngt, :), ...
            AO.Sail1.Sail_grav(1:lngt, :), AO.Sail1.Sail_wind(1:lngt, :)];
        
end

end