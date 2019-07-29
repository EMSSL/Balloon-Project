function [Cl, Cd, ai, cmc4, chord, yLE, Fx, Fy, Fz, yl, Mc4] = ReadStrips(filename, plotFlag, varargin)
% [Cl, Cd, ai, cmc4] = ReadStrips(filename) returns the section lift
% coefficient, the section drag coefficient, angle of attack, and moment
% about the quarter chord for the strips in filename. 
%
% INPUTS:
%   filename    - the .strip file generated from AVL code
%   plotFlag    - 1 == yes, plot results. 0 == no plot
% 
% OUTPUTS:
%   Cl          - n x b matrix of sectional lift coefficients
%   Cd          - n x b matrix of sectional drag coefficients
%   ai          - n x b matrix of local angle of attack
%   cmc4        - n x b matrix of moment about the quarter chord
%   chord       - n x b matrix of chord lengths
%   yLE         - n x b matrix of leading edges in span direction
%   Fx          - n x b matrix of X direction forces per unit span
%       CURRENTLY: This is using Cd since drag is in +VE X direction
%   Fy          - n x b matrix of Y direction forces per unit areaspan
%       CURRNETLY: This returns NaN matrix. Use side force coefficients to
%       determine side forces in future revision. 
%   Fz          - n x b matrix of Z direction forces per unit areaspan
%       CURRENTLY: This is using Cl since lift is in +VE Z direction
%   Mc4         - n x b matrix of airfoil pitching moment per unit span
%   yl          - center of lift for the sail in spanwise direction
%
% VARARGIN:
%   design structure populated from RunDesign.m with all the info needed
% 
% NOTE: n is the number of surfaces defined, b is the number of spanwise
% voritces
% NOTE: the wing and geometry plot only is valid for hershey bar wings
%
% Modification table:
% 1. CDY, 9/12/17, added in ability to calculate forces on the bodies from
%       strips file and input structure

close all

% check varargin
rho = NaN;
vel = NaN;
if isempty(varargin) ~= 1   
    design = varargin{1};        % overall structure
    if isstruct(design) == 1
        wingS = design.wing;         % wing structure in design structure
        rudderS = design.rudder;     % rudder structure in design structure
        runsS = design.runs;
        rho = runsS.rho;     % density
        vel = runsS.vel;     % velocity
    end
% 
%     
%     switch exist(design)
%         case 0
%             error('The design you have specified does not exist in the current working directory.');
%     
%         case 1
%             
%             
%         case 2
%             s1 = load(design);
%             designm = s1.design;
%             wingS = designm.wing;         % wing structure in design structure
%             rudderS = designm.rudder;     % rudder structure in design structure
%             runsS = designm.runs;
%             rho = runsS.rho;     % density
%             vel = runsS.vel;     % velocity
%             
%         otherwise
%             error('Input design is not of structure or matrix file format.');
%     end
    
    
end

% preallocate
numc = 2000;
Cl = NaN(numc, numc);
Cd = NaN(numc, numc);
ai = NaN(numc, numc);
cmc4 = NaN(numc, numc);
chord = NaN(numc, numc);
yLE = NaN(numc, numc);
nmbr = NaN(numc, numc);

% % open and discard headers
% fID = fopen(filename);
% for i1 = 1:7
%     newLine = fgetl(fID);
% end
% 
% % get all the surfaces
% Ncntr = 1;
% Bcntr = 1;
% while newLine ~= -1
%     
%     % handle next line
%     a = strsplit(newLine);      % parse next line
%     if length(a) >= 2           % if greater than two
%         
%         % header for a new surface
%         if strcmp(a{3}, '#') == 1   % if pound sign
%             names{Ncntr} = a{5};     % get name
%             
%             newLine = fgetl(fID);   % get next line
%             a = strsplit(newLine);  % split new line
%             indx(Ncntr) = str2double(a{9});
%             
%             newLine = fgetl(fID);   % get next line
%             a = strsplit(newLine);  % split new line
%             Sref(Ncntr) = str2double(a{5});
%             crd = str2double(a{9});
%             spn(Ncntr) = Sref(Ncntr)/crd;
%             
%             newLine = fgetl(fID);
%             exit1 = 0;
%             while exit1 ~= 1
%                 a = strsplit(newLine);
%                 if length(a) == 8 && strcmp(a{2}, 'Strip') == 1
%                     newLine = fgetl(fID);
%                     exit1 = 1;
%                 else
%                     newLine = fgetl(fID);
%                 end
%             end
%             newLine = fgetl(fID);
%             
%             for i1 = 1:indx(Ncntr)
%                 a = strsplit(newLine);
%                 Cl(Bcntr, Ncntr) = str2double(a{9});
%                 Cd(Bcntr, Ncntr) = str2double(a{10});
%                 ai(Bcntr, Ncntr) = str2double(a{7});
%                 cmc4(Bcntr, Ncntr) = str2double(a{12});
%                 chord(Bcntr, Ncntr) = str2double(a{4});
%                 yLE(Bcntr, Ncntr) = str2double(a{3});
%                 nmbr(Bcntr, Ncntr) = str2double(a{2});
%                 Bcntr = Bcntr + 1;
%                 newLine = fgetl(fID);
%             end
%             Ncntr = Ncntr + 1;
%             Bcntr = 1;
%             
%             
%                    
%         end
%         
%         
%     end
%     newLine = fgetl(fID);
%     
% end


fID = fopen(filename);  % open file
newLine = fgetl(fID);   % get new line
nCNTR = 0;
bCNTR = 1;

% while loop
while newLine ~= -1
    
    if length(newLine) > 7
        
        % consider lines
        switch newLine(1:7)

            % surface and strip forces lines
            case ' Surfac'
                for i1 = 1:4
                    newLine = fgetl(fID);
                end
            
            % surface 
            case '  Surfa'
                nCNTR = nCNTR + 1;
                a = strsplit(newLine);
                names = a{5};           % wing or rudder
                newLine = fgetl(fID);   % chord line
                newLine = fgetl(fID);   % surface area and chord
                a = strsplit(newLine);
                Sref(nCNTR) = str2num(a{5});
                Chord(nCNTR) = str2num(a{9});
                
                for i1 = 1:9
                    newLine = fgetl(fID);
                end
            
            % strip forces
            case ' Strip '
                newLine = fgetl(fID);
                newLine = fgetl(fID);
                while newLine ~= -1
                    a = strsplit(newLine);
                    Cl(bCNTR, nCNTR) = str2num(a{9});
                    Cd(bCNTR, nCNTR) = str2num(a{10});
                    ai(bCNTR, nCNTR) = str2num(a{7});
                    cmc4(bCNTR, nCNTR) = str2num(a{12});
                    chord(bCNTR, nCNTR) = str2num(a{4});
                    yLE(bCNTR, nCNTR) = str2num(a{3});
                    nmbr(bCNTR, nCNTR) = str2num(a{2});
                    bCNTR = bCNTR + 1;
                    newLine = fgetl(fID);
                    if length(newLine) > 7 
                        if strcmp(newLine(1:7), ' ------') == 1
                            newLine = -1;
                        end
                    end
                        
                end
                bCNTR = 1;
                
        end
        
    end
    
    % next line
    newLine = fgetl(fID);

end



% plotting the results
maxROW = numc - sum(isnan(Cl(:, 1)));
% maxROW = max(indx)     % most number of sections
maxCOL = nCNTR;     % number of sections
Cl = Cl(1:maxROW, 1:maxCOL);
Cd = Cd(1:maxROW, 1:maxCOL);
ai = ai(1:maxROW, 1:maxCOL);
cmc4 = cmc4(1:maxROW, 1:maxCOL);
chord = chord(1:maxROW, 1:maxCOL);
yLE = yLE(1:maxROW, 1:maxCOL);
nmbr = nmbr(1:maxROW, 1:maxCOL);



% add in ability to figure out clmax for a given situation
% clmax = 1.0;
clmax = getCLmax(design, yLE);



% draw surface results
if plotFlag == 1
    figure;
    hold on
    MarkerSize = 10;
    LineWidth = 1.5;
    mrkrs = {'-o'; '-s'; '-v'; '-p'; '->'; '-d'; '-h'; '-^'; '-x'; '-<'};
    for i1 = 1:1
%     for i1 = 1:maxCOL
%         plot(yLE(:, i1), Cl(:, i1), mrkrs{i1}, 'MarkerSize', ...
%             MarkerSize', 'LineWidth', LineWidth)
        % lege1{i1} = names{i1};
        plot(yLE(:, i1)/2, Cl(:, i1), 'LineWidth', LineWidth);
    end
    % plot(yLE(:, 1)/2, clmax*ones(length(yLE(:, 1))), 'r', 'LineWidth', LineWidth)
    plot(yLE(:, 1)/2, clmax(:, 1), 'r', 'LineWidth', LineWidth)
    xlabel('Span')
    ylabel('c$_{l}$')
    grid on
    yt = yLE(:, 1);
    xlim([yt(1), 0]/2);
    ylim([0, 2])
    legend({'c$_{l}$'; 'c$_{l_{max}}$'}, 'interpreter', 'latex', 'location', 'southeast')
    % keyboard
    % lege1{i1 + 1} = 'c$_{l_{max}}$';
    % legend(lege1, 'interpreter', 'latex')

%     figure(2);
%     hold on
%     plot(-50, -50, 'o', 'MarkerSize', MarkerSize, 'Color', ...
%                     [0    0.4470    0.7410])
%     plot(-50, -50, 'x', 'MarkerSize', MarkerSize, 'Color', ...
%                     [0.8500    0.3250    0.0980])
%     legend({'Normal', 'Stall'}, 'interpreter', 'latex', ...
%         'location', 'southeast')
%     xIncr = 0;
%     for i1 = 1:maxCOL
%         for i2 = 1:maxROW
% 
%             if Cl(i2, i1) > clmax
%                 plot(xIncr + chord(i2, i1)/4, yLE(i2, i1), ...
%                     'x', 'MarkerSize', 4, 'Color', ...
%                     [0.8500    0.3250    0.0980])
%                     
%             else
%                 plot(xIncr + chord(i2, i1)/4, yLE(i2, i1), ...
%                     'o', 'MarkerSize', 4, 'Color', ...
%                     [0    0.4470    0.7410])
%             end
% 
%         end
% 
%         % plot wing outlines - only works with rectangular wings
%         plot(xIncr + [0, 0, chord(1, i1), chord(1, i1), 0], ...
%             [0, -spn(i1), -spn(i1), 0, 0], '-k', 'LineWidth', LineWidth)
% 
%         xIncr = xIncr + 3.5;
% 
%     end
%     
%     axis([-1, xIncr - 3, -1.1*max(spn), 1])
%     xlabel('Chord Direction')
%     ylabel('Span Direction')
%     grid on
    
end


% calculate forces and moments on the system
Fz = 0.5*rho*(vel^2)*Cl.*chord;     % N/m, lift per unit span
Fx = 0.5*rho*(vel^2)*Cd.*chord;     % N/m, drag per unit span
Fy = NaN*Cl;
Mc4 = 0.5*rho*(vel^2)*cmc4.*chord.*chord;   % N, pitching moment per unit span


% calculate yl
num = sum(Cl(:, 1).*yLE(:, 1));
den = sum(Cl(:, 1));
yl = num/den;

% figure(1);
% hold on
% plot(yl*[1, 1], clmax*[0, 1], 'k*-')
% set(gcf, 'Position', [2995         406         560         420]);


end