% Script to process, plot data
% CDY, 8/11/2018

% preamble
clc                                 % clear command line
close all                           % close all figures
clear                               % clear all variables
fclose('all');                      % close all text files
UsePackage('add', 'GenPackage');    % comment out unless you're CDY
SetDefaults;                        % comment out unless you're CDY

% file to process
filename = 'sampleData.txt';        % file to read data
M = csvread(filename, 5, 0);        % get csv data to matrix

% plot wind speed
figure
plot(M(:, 1)/1000, M(:, 2), 'LineWidth', 1);
xlabel('Time [s]');
ylabel('Wind speed [m/s]');
grid on 
grid minor

% plot wind direction bounded by [0, 360]
figure
plot(M(:, 1)/1000, M(:, 3), 'LineWidth', 1);
xlabel('Time [s]');
ylabel('Wind direction [deg]');
grid on 
grid minor