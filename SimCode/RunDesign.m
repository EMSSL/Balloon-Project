% Main file to run AVL based on desired geometry

% preamble
clc
close all
clear
UsePackage('add', 'GenPackage');
SetDefaults;

% name
avlLocation = '.\avl.exe';
runs.header = 'TCS AUTO';   % name of run within files
runs.name = 'tcs_auto';     % name of files without extensions
showStall = 1;              % stall plot flag, 1 == plot, 0 == no plot
delFlag = 1;                % delete other files to prevent interference

% wing parameters
wing.span = 7;      % [m] span
wing.chord = 0.7;	% [m] chord
wing.n_cho = 8;     % number of chordwise vortices
wing.n_spa = 16;    % number of spanwise vortices
wing.n_ail = 0;     % number of ailerons
wing.d_ail = 0;     % [deg] aileron deflection
wing.xle = 0;       % [m] offset of leading edge 
wing.yle = 0;       % [m] offset of leading edge 
wing.zle = 0;       % [m] offset of leading edge 
wing.mass = 30;     % [kg] wing mass
wing.IXX = 1;       % [kg-m2]
wing.IYY = 1;       % [kg-m2]
wing.IZZ = 1;       % [kg-m2]
wing.IXY = 1;       % [kg-m2]
wing.IXZ = 1;       % [kg-m2]
wing.IYZ = 1;       % [kg-m2]

% rudder parameters
rudder.span = 3;      % [m] span
rudder.chord = 0.3;	% [m] chord
rudder.n_cho = 8;     % number of chordwise vortices
rudder.n_spa = 16;    % number of spanwise vortices
rudder.d_ele = 0;     % [deg] aileron deflection
rudder.xle = 3.5*wing.chord;       % [m] offset of leading edge 
rudder.yle = 0;       % [m] offset of leading edge 
rudder.zle = 0;       % [m] offset of leading edge 
rudder.mass = 3;     % [kg] wing mass
rudder.IXX = 1;       % [kg-m2]
rudder.IYY = 1;       % [kg-m2]
rudder.IZZ = 1;       % [kg-m2]
rudder.IXY = 1;       % [kg-m2]
rudder.IXZ = 1;       % [kg-m2]
rudder.IYZ = 1;       % [kg-m2]
rudder.xbc_ele = 0.7;   % x/c for elevator 

% run parametrs
runs.Lunit = 1;     % [m] length unit
runs.Munit = 1;     % [kg] mass unit
runs.Tunit = 1;     % [s] time unit
runs.mach = 0;
runs.Xsim = 0;
runs.Ysim = 0;
runs.Zsim = 0;
runs.CDp = 0.025;   % profile drag coefficient
runs.Sref = wing.chord*wing.span;
runs.Cref = wing.chord;
runs.Bref = wing.span;
runs.g = 9.81;
runs.rho = 1.225;
runs.vel = 1;
runs.stFlag = 1;    % Save ST file, 1 == yes, 0 == no
runs.fsFlag = 1;    % Save fs file, 1 == yes, 0 == no


% aerodynamic constraints
const.CL = 1;       % lift coefficient constraint
const.b = 0;        % beta
const.r = 0;        % roll rate
const.p = 0;        % pitch rate
const.y = 0;        % yaw rate
runs.const = const; % generic constraints


% update these with actual relationships for NACA 0015
wing.xcg = wing.chord/2 + wing.xle;
wing.ycg = wing.span/2 + wing.yle;
wing.zcg = 0 + wing.zle;
rudder.xcg = rudder.chord/2 + rudder.xle;
rudder.ycg = rudder.span/2 + rudder.yle;
rudder.zcg = 0 +  + rudder.zle;

xcg = ((wing.chord/2 + wing.xle)*wing.mass + (rudder.chord/2 + ...
    rudder.xle)*rudder.mass)/(rudder.mass + wing.mass);
ycg = ((wing.span/2 + wing.yle)*wing.mass + (rudder.span/2 + ...
    rudder.yle)*rudder.mass)/(rudder.mass + wing.mass);
zcg = 0;

runs.Xref = xcg;
runs.Yref = ycg;
runs.Zref = zcg;



% % %       LEAVE THIS ALONE



% compile structure and send out
design.wing = wing;         % save wing structure in design structure
design.rudder = rudder;     % save rudder structure in design structure
design.runs = runs;         % save runs structure in design structure

% write the files
if delFlag == 1
    [status, result] = dos(['del ', runs.name, '.stabs']);
    [status, result] = dos(['del ', runs.name, '.strip']);
    [status, result] = dos(['del ', runs.name, '.mass']);
    [status, result] = dos(['del ', runs.name, '.avl']);
end
[avlFile, massFile, runFile] = WriteAVL(design);
fclose('all');

% % run once to get the stability derivatives
[status, result] = dos([avlLocation,' < ', runFile]);

% % run and report all the stuff avl prints out (troubleshooting)
% [status, result] = dos([avlLocation,' < ', runFile], '-echo');

% % % % RUN MATLAB CODE to find deflections, forces, etc.

% read in stability derivatives
if runs.stFlag == 1
    stabDerivs = ReadStabDerivs([runs.name, '.stabs']);
end

% read in strips
if runs.fsFlag == 1
    [Cl, Cd, ai, cmc4, chord, yLE] = ReadStrips([runs.name, '.strip'], 1);
end