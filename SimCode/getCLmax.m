function [Clmax] = getCLmax(design, yLE)
% [Clmax] = getCLmax(design) returns a n x 1 array of Cl max values based
% on design. 

% pull datas
% assume Re = 1e5 constraints
% add real data later
% datas = [0,     1.1;
%          15,    1.6;
%          20,    1.4];
% Cl0 = 1.1;

% real data for x/c = 0.7 flap
datas = [0,          1.1357;
         2.0000,     1.1970;
         4.0000,     1.2581;
         6.0000,     1.3168;
         8.0000,     1.3745;
         10.0000,    1.4291;
         12.0000,    1.4700;
         14.0000,    1.4979;
         16.0000,    1.5391;
         18.0000,    1.5824];
Cl0 = datas(1, 2);

% get design stuffs
wing = design.wing;
aildef = wing.d_ail;    % [deg] aileron deflection
fpdef = wing.d_fps;     % [deg] flap deflection
spn = wing.n_spa;
ailS = wing.s_ail;
ailE = wing.e_ail;
fp1S = wing.s_fps(1);
fp2S = wing.s_fps(2);
fp3S = wing.s_fps(3);
fp1E = wing.e_fps(1);
fp2E = wing.e_fps(2);
fp3E = wing.e_fps(3);


% set flap values along span
Clmax = NaN(length(yLE), 1);
yLE = -yLE;
for i1 = 1:length(yLE)
    
    % aileron
    if ailS <= yLE(i1) && yLE(i1) <= ailE
        Clmax(i1) = interp1(datas(:, 1), datas(:, 2), abs(aildef));
        
    % flap 1
    elseif fp1S <= yLE(i1) && yLE(i1) <= fp1E
        Clmax(i1) = interp1(datas(:, 1), datas(:, 2), abs(fpdef(1)));
        
    % flap 2
    elseif fp2S <= yLE(i1) && yLE(i1) <= fp2E
        Clmax(i1) = interp1(datas(:, 1), datas(:, 2), abs(fpdef(2)));
    
    % flap 3
    elseif fp3S <= yLE(i1) && yLE(i1) <= fp3E
        Clmax(i1) = interp1(datas(:, 1), datas(:, 2), abs(fpdef(3)));
    
    else
        Clmax(i1) = Cl0;
    
    end
    
end
    
end