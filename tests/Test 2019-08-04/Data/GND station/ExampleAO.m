% Example code to read in data files

clc
close all
clear

% GPS data
% startline = 2166, line where actual data starts
GPSdata = ParseFile('GPSLOG00.TXT', 'startline', 280);

% AO data
% startline = 5362, line where actual data starts
AOdata = ParseFile('AODATA00.TXT', 'startline', 1234);

% plot the GPS datas
figure('Color', 'w')
GPGGAtime = GPSdata.GPGGA(:, end) - GPSdata.GPGGA(1, end);
GPRMCtime = GPSdata.GPRMC(:, end) - GPSdata.GPRMC(1, end);
subplot(4, 1, 1)
hold on
plot(GPGGAtime, GPSdata.GPGGA(:, 4));
plot(GPRMCtime, GPSdata.GPRMC(:, 4));
ylabel('Latitude [deg]');
grid on
subplot(4, 1, 2)
hold on
plot(GPGGAtime, GPSdata.GPGGA(:, 5));
plot(GPRMCtime, GPSdata.GPRMC(:, 5));
ylabel('Longitude [deg]');
legend('GPGGA', 'GPRMC');
grid on
subplot(4, 1, 3)
plot(GPGGAtime, GPSdata.GPGGA(:, 7));
ylabel('# of satellites');
grid on
subplot(4, 1, 4)
plot(GPGGAtime, GPSdata.GPGGA(:, 8));
xlabel('Elapsed time [s]');
ylabel('Altitude [m]');
grid on

% plot the AO data
figure('Color', 'w')
AOtime = AOdata.AO(:, end) - AOdata.AO(1, end);
subplot(4, 2, 1)
hold on
plot(AOtime, AOdata.AO(:, 6));
plot(AOtime, AOdata.AO(:, 7));
plot(AOtime, AOdata.AO(:, 8));
ylabel('Acceleration [m/s]');
legend('X', 'Y', 'Z');
grid on
subplot(4, 2, 2)
hold on
plot(AOtime, AOdata.AO(:, 9));
plot(AOtime, AOdata.AO(:, 10));
plot(AOtime, AOdata.AO(:, 11));
ylabel('Angular velocity [rad/s]');
legend('X', 'Y', 'Z');
grid on
subplot(4, 2, 3)
hold on
plot(AOtime, AOdata.AO(:, 12));
plot(AOtime, AOdata.AO(:, 13));
plot(AOtime, AOdata.AO(:, 14));
ylabel('Magnetic fields [uT]');
legend('X', 'Y', 'Z');
grid on
subplot(4, 2, 4)
hold on
plot(AOtime, AOdata.AO(:, 15));
plot(AOtime, AOdata.AO(:, 16));
plot(AOtime, AOdata.AO(:, 17));
ylabel('Euler angles [deg]');
legend('X', 'Y', 'Z');
grid on
subplot(4, 2, 5)
hold on
plot(AOtime, AOdata.AO(:, 18));
plot(AOtime, AOdata.AO(:, 19));
plot(AOtime, AOdata.AO(:, 20));
ylabel('Gravity [m/s2]');
grid on
legend('X', 'Y', 'Z');
subplot(4, 2, 6);
plot(AOtime, AOdata.AO(:, 21));
ylabel('Temperature [degC]');
grid on
subplot(4, 2, 7)
plot(AOtime, AOdata.AO(:, 24));
ylabel('Altitude [m]');
grid on
xlabel('Elapsed time [s]');
subplot(4, 2, 8)
plot(AOtime, AOdata.AO(:, 25));
ylabel('Wind speed [m/s]');
grid on
xlabel('Elapsed time [s]');