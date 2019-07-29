% Main file to run AVL based on desired geometry

% preamble
clc
close all
clear
% UsePackage('add', 'GenPackage', 'OptPackage');
% SetDefaults;

% name
avlLocation = '.\avl.exe';
runs.header = 'TCS_opt';   % name of run within files

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
runs.name = '';     % name of files without extensions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
showStall = 0;              % stall plot flag, 1 == plot, 0 == no plot
delFlag = 1;                % delete other files to prevent interference

% FACTOR = 2
% subscale wing parameters
wing.span = 1.6;          % [m] span
wing.chord = 0.32;          % [m] chord
wing.n_cho = 7;             % number of chordwise vortices
wing.n_spa = 147;            % number of spanwise vortices
wing.n_ail = 1;             % number of ailerons - control rolling moment
wing.d_ail = 0;             % [deg] aileron deflection
wing.s_ail = 0.908;         % [m] start of ail on span 
wing.e_ail = 1.196;         % [m] end point on span moving from 0 on root to tip
wing.xbc_ail = 0.7;         % x/c for aileron
wing.n_fps = 3;             % number of flaps - control lift distribution
wing.d_fps = [0, 0, 0];     % [deg] flap deflection, [boom to tip flaps][F1, F2, F3] 
wing.xbc_fps = 0.7;                     % x/c flap locations
wing.s_fps = [0.01, 0.308, 0.608];     % [m] start of the flaps
wing.e_fps = [0.298, 0.598, 0.898];     % [m] end of the flaps
wing.xle = 0.0;           % [m] offset of leading edge 
wing.yle = 0;               % [m] offset of leading edge 
wing.zle = 0;               % [m] offset of leading edge 
wing.mass = 0.5099;            % [kg] wing mass + boom mass assumption 


% rudder parameters
rudder.span = 0.554;     % [m] span
rudder.chord = 0.185;   % [m] chord
rudder.n_cho = 7;       % number of chordwise vortices
rudder.n_spa = 37;      % number of spanwise vortices
rudder.d_ele = 0;       % [deg] aileron deflection
rudder.xle = wing.chord*3.5;     % [m] offset of leading edge 
rudder.yle = 0;         % [m] offset of leading edge 
rudder.zle = 0;         % [m] offset of leading edge 
rudder.mass = 0.0509;      % [kg] wing mass
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
runs.vel = 6.1676;  % Re ~= 103,053.1191
runs.stFlag = 1;    % Save ST file, 1 == yes, 0 == no
runs.fsFlag = 1;    % Save fs file, 1 == yes, 0 == no

% 
% % % ADD IN THE CALCULATIONS FOR IXX IYY etc.



% % electronics package
% runs.elecs = 0.020;     % [kg] ballast mass
% runs.elecsCG = [wing.xle + wing.chord/2, wing.yle - ...
%     wing.span*0.98, wing.zle];
% runs.elecsI = [1, 1, 1, 0, 0, 0];   % [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]


% aerodynamic constraints
const.b = 0;        % beta
const.r = 0;        % roll rate
const.p = 0;        % pitch rate
const.y = 0;        % yaw rate
runs.const = const; % generic constraints


% % update these with actual relationships for NACA 0015
% wing.xcg = wing.chord/2 + wing.xle;
% wing.ycg = wing.span/2 + wing.yle;
% wing.zcg = 0 + wing.zle;
% rudder.xcg = rudder.chord/2 + rudder.xle;
% rudder.ycg = rudder.span/2 + rudder.yle;
% rudder.zcg = 0 +  + rudder.zle;
% 
% xcg = ((wing.chord/2 + wing.xle)*wing.mass + (rudder.chord/2 + ...
%     rudder.xle)*rudder.mass)/(rudder.mass + wing.mass);
% ycg = ((wing.span/2 + wing.yle)*wing.mass + (rudder.span/2 + ...
%     rudder.yle)*rudder.mass)/(rudder.mass + wing.mass);
% zcg = 0;

% run values - from cm_calc.m
runs.IXX = 0.0039;       % [kg-m2]
runs.IYY = 0.0142;       % [kg-m2]
runs.IZZ = 0.1015;       % [kg-m2]
runs.IXY = 0.1214;       % [kg-m2]
runs.IXZ = 0;       % [kg-m2]
runs.IYZ = 0;       % [kg-m2]
runs.Xref = xcg;
runs.Yref = ycg;
runs.Zref = zcg;
runs.mass = 1.3169; % [kg] sail mass


% % %       LEAVE THIS ALONE

% disp('Loading parameters ... ');
% error checking
if length(wing.d_fps) ~= wing.n_fps
    error(['Length of wing.d_fps (', num2str(wing.d_fps), ...
        ') is not equal to the number of flaps (', ...
        num2str(wing.n_fps), ')']);
end

% compile structure and send out
design.wing = wing;         % save wing structure in design structure
design.rudder = rudder;     % save rudder structure in design structure
design.runs = runs;         % save runs structure in design structure
% GeoDraw(design);
% keyboard

% % % find the stall conditions
% THIS CODE WHEN UNCOMMENTED FINDS THE STALL POINT AS DEFINED AS THE SAIL
% CL_i = CL_max FOR THE AIRFOIL
% [alfa, CL, errs, iters] = findStall(design, 1)
% figure(1);
% close(gcf);

% perform alpha run
% alpha = -5:1:5;   % run alphas for coefficient comparison
% alpha = 0:0.75:10;  % run alphas for roll angle comparison
alpha = -15:2.5:15;
[CL, CD, xnp, yL] = alphaRun_dynamicSIMS(design, alpha);

% plot the coefficients vs alpha
figure(1);
plot(alpha, CL, '-o', alpha, CD, '-^')
xlabel('$\alpha$ [deg]');
ylabel('Coefficients');
grid on
legend({'$C_L$', '$C_D$'}, 'interpreter', 'latex', 'location', 'southeast');
title('AVL @ Re = 103,053.12');

% plot the neutral point and center of lift coordinate
figure(2);
plot(alpha, xnp, '-o', alpha, yL, '-^')
xlabel('$\alpha$ [deg]');
ylabel('Coordinates');
grid on
legend({'$x_{np}$', '$y_L$'}, 'interpreter', 'latex', 'location', 'southeast');
title('AVL @ Re = 103,053.12');

% % save all the data
%
% % save the no flaps design for 2x subscale model
% save('AVLrun2xNoFlaps.mat', 'alpha', 'CL', 'CD', 'xnp', 'design', 'yL')