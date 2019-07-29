function [avlFile, massFile, runFile] = WriteAVL_C(design)
% [avlFile, massFile, runFile] = WriteAVL(design) returns the .avl, .mass, 
% and .run files for files for the given structure design. 
%
% NOTE: The following files must be present within the folder:
%   template.avl
%   template.mass
%
% INPUTS:
%   design  - a structure with the given design values
%
% FIELDS:
%   wing    -   a structure with wing parameters
%   rudder  -   a structure with rudder parameters
%   runs    -   a structure of run conditions
%   header  -   a string for a header
%   name    -   name of the files (run case) without extensions
%
%   WING:
%       span    - span of the wing
%       chord   - chord of the wing
%       n_cho   - number of chordwise vortices
%       n_spa   - number of spanwise vortices
%       n_ail   - number of ailerons
%       d_ail   - n_ail x 1 vector of aileron deflections
%       xle     - leading edge x offset (should be zero)
%       yle     - leading edge y offset (should be zero)
%       zle     - leading edge z offset (should be zero)
%       mass    - mass of the wing
%       Ixx     - moment of inertia about CM x axis
%       IYY     - moment of inertia about CM y axis
%       IZZ     - moment of inertia about CM z axis
%       IXY     - product of inertia about xy
%   	IXZ     - product of inertia about xz
%   	IYZ     - product of inertia about yz
%
%   RUDDER:
%       span    - span of the rudder
%       chord   - chord of the rudder
%       n_cho   - number of chordwise vortices
%       n_spa   - number of spanwise vortices
%       d_ele   - elevator deflections
%       xle     - leading edge x offset (should be non-zero)
%       yle     - leading edge y offset (should be zero)
%       zle     - leading edge z offset (should be zero)
%       mass    - mass of the rudder
%       Ixx     - moment of inertia about CM x axis
%       IYY     - moment of inertia about CM y axis
%       IZZ     - moment of inertia about CM z axis
%       IXY     - product of inertia about xy
%   	IXZ     - product of inertia about xz
%   	IYZ     - product of inertia about yz
%
% OUTPUTS:
%   avlFile     - .avl file to import geometry
%   massFile    - .mass file to incorporate the mass of the system
%   runFile     - .run file to execute avl from Matlab script

% Written by Christopher D. Yoder on 9/1/2017 to run avl from matlab. 
% 
% % error handle
% if exist('template.avl', 'file') ~= 2
%     error('template.avl not found in the local directory.');
%     
% elseif exist('template.mass', 'file') ~= 2
%     error('template.mass not found in the local directory.');
%     
% end

% get stuffs
wing = design.wing;             % wing structure
rudder = design.rudder;         % rudder structure
runs = design.runs;             % runs structure
linewrite = '! *** *** Saurabh is AWESOME :D *** ***';



% write avl file
avlFile = [runs.name, '.avl'];  % make file name
fileAVL = fopen(avlFile, 'w');  % create file

% header lines
fprintf(fileAVL, '%s \n', linewrite);        % first line
fprintf(fileAVL, '! %s \n', runs.header);      % write header line

% preamble
WritePreamble(fileAVL, runs.mach, [runs.Xsim, runs.Ysim, runs.Zsim], ...
    [runs.Sref, runs.Cref, runs.Bref], ...
    [runs.Xref, runs.Yref, runs.Zref], runs.CDp, linewrite);



% wing surface
WriteSurface(fileAVL, 'WING', wing.n_cho, 0.0, wing.n_spa, 0.0, linewrite); % _A, total vortices
% WriteSurface(fileAVL, 'WING', wing.n_cho, 0.0, [], [], linewrite); % _B, individual vortices

% wing left section
WriteSection(fileAVL, wing.xle, -wing.span + wing.yle, wing.zle, ...
    wing.chord, 0.0, [], [], 'NACA 0015');  % _A, total vortices
% WriteSection(fileAVL, wing.xle, -wing.span + wing.yle, wing.zle, ...
%     wing.chord, 0.0, wing.n_spa, 1.0, 'NACA 0015'); % _B, individual vortices

% write ailerons
n_ail = wing.n_ail;                 % number of ailerons
if n_ail ~= 0
    fprintf(fileAVL, '%s \n', linewrite);
    fprintf(fileAVL, '%s \n', linewrite);
    
%     % write far flap section
    WriteSection(fileAVL, wing.xle, wing.yle - wing.e_ail, wing.zle, ...
        wing.chord, 0.0, [], [], []);
%     WriteSection(fileAVL, wing.xle, wing.yle - wing.e_ail, wing.zle, ...
%         wing.chord, 0.0, wing.n_spa, 1.0, []);
    nme = 'ailer';
    WriteControl(fileAVL, nme, 1.0, wing.xbc_fps, [0.0, 0.0, 0.0], -1.0)
    
    % write near flap section
    WriteSection(fileAVL, wing.xle, wing.yle - wing.s_ail, wing.zle, ...
        wing.chord, 0.0, [], [], []); % _A, total vortices
%     WriteSection(fileAVL, wing.xle, wing.yle - wing.s_ail, wing.zle, ...
%         wing.chord, 0.0, wing.n_spa, 1.0, []);
    WriteControl(fileAVL, nme, 1.0, wing.xbc_fps, [0.0, 0.0, 0.0], 1.0)
end    
        
% write flaps
n_fps = wing.n_fps;                 % number of flaps
if n_fps ~= 0
    for i1 = 1:n_fps                % 1, 2, 3
        fprintf(fileAVL, '%s \n', linewrite);
        fprintf(fileAVL, '%s \n', linewrite);
        indx = n_fps - i1 + 1;      % 3, 2, 1
        
        % write far flap section
        WriteSection(fileAVL, wing.xle, wing.yle - wing.e_fps(indx), ...
            wing.zle, wing.chord, 0.0, [], [], []);
%         WriteSection(fileAVL, wing.xle, wing.yle - wing.e_fps(indx), ...
%             wing.zle, wing.chord, 0.0, wing.n_spa, 1.0, []);
        nme = ['flap', num2str(indx)];
        % nme = ['flap', num2str(i1)];
        WriteControl(fileAVL, nme, 1.0, wing.xbc_fps, [0.0, 0.0, 0.0], 1.0)

        % write near flap section
        WriteSection(fileAVL, wing.xle, wing.yle - wing.s_fps(indx), ...
            wing.zle, wing.chord, 0.0, [], [], []);
%         WriteSection(fileAVL, wing.xle, wing.yle - wing.s_fps(indx), ...
%             wing.zle, wing.chord, 0.0, wing.n_spa, 1.0, []);
        WriteControl(fileAVL, nme, 1.0, wing.xbc_fps, [0.0, 0.0, 0.0], 1.0)
    end           
end
fprintf(fileAVL, '%s \n', linewrite);
fprintf(fileAVL, '%s \n', linewrite);
    
% wing right section
WriteSection(fileAVL, wing.xle, wing.yle, wing.zle, ...
    wing.chord, 0.0, [], [], []);
% WriteSection(fileAVL, wing.xle, wing.yle, wing.zle, ...
%     wing.chord, 0.0, wing.n_spa, 1.0, []);




% rudder surface
WriteSurface(fileAVL, 'RUDDER', rudder.n_cho, 0.0, rudder.n_spa, 0.0, linewrite);
% WriteSurface(fileAVL, 'RUDDER', rudder.n_cho, 1.0, rudder.n_spa, 1.0, linewrite);

% rudder left section
WriteSection(fileAVL, rudder.xle, -rudder.span - rudder.yle, rudder.zle, ...
    rudder.chord, 0.0, [], [], 'NACA 0015');

% rudder elevator
WriteControl(fileAVL, 'ELEV', 1.0, rudder.xbc_ele, ...
    [0.0, 0.0, 0.0], 1.0)

% rudder right section
WriteSection(fileAVL, rudder.xle, rudder.yle, rudder.zle, ...
    rudder.chord, 0.0, [], [], []);

% rudder elevator
WriteControl(fileAVL, 'ELEV', 1.0, rudder.xbc_ele, ...
    [0.0, 0.0, 0.0], 1.0)

% close file
fclose(fileAVL);



% write mass file
massFile = [runs.name, '.mass'];  % make file name
fileMASS = fopen(massFile, 'w');  % create file

% write preamble
WritePreambleMass(fileMASS, runs.Lunit, runs.Munit, runs.Tunit, ...
    runs.g, runs.rho)

% write mass table - wing
WriteMassTable(fileMASS, wing.mass, wing.xcg, wing.ycg, wing.zcg, ...
    wing.IXX, wing.IYY, wing.IZZ, wing.IXY, wing.IXZ, wing.IYZ, 'wing'); 

% write mass table - rudder
WriteMassTable(fileMASS, rudder.mass, rudder.xcg, ...
    rudder.ycg, rudder.zcg, rudder.IXX, rudder.IYY, ...
    rudder.IZZ, rudder.IXY, rudder.IXZ, rudder.IYZ, 'rudder'); 

% write mass table - electronics sled
WriteMassTable(fileMASS, runs.elecs, runs.elecsCG(1), runs.elecsCG(2), ...
    runs.elecsCG(3), runs.elecsI(1), runs.elecsI(2), runs.elecsI(3), ...
    runs.elecsI(4), runs.elecsI(5), runs.elecsI(6), 'electronics'); 

% close file
fclose(fileMASS);

%%%%%%%%%%%%%%%%%%%%%%%%%
% enter deflections
n_tot = n_fps + n_ail + 1;  % total number of control surfaces
num1 = ['d', num2str(n_tot), ' '];
elevn = num2str(n_tot);
%%%%%%%%%%%%%%%%%%%%%%%%%

% write run file
runFile = [runs.name, '.run'];  % make file name
fileRUN = fopen(runFile, 'w');  % create file

% load written files
fprintf(fileRUN, 'LOAD %s\n', avlFile);     % load avl file
fprintf(fileRUN, 'MASS %s\n', massFile);    % load mass file
fprintf(fileRUN, 'MSET\n');
fprintf(fileRUN, '%i\n', 0);

% disable graphics
fprintf(fileRUN, 'PLOP\ng\n\n');

% enter run case settings
fprintf(fileRUN, '%s\n', 'OPER');                   % oper menu
fprintf(fileRUN, '%s\n', 'm');                      % edit parameters
fprintf(fileRUN, '%s%6.4f\n', 'v ', runs.vel);      % velocity
fprintf(fileRUN, '%s%6.4f\n', 'mn ', runs.mach);    % mach number
fprintf(fileRUN, '%s%6.4f\n', 'g ', runs.g);        % gravity
fprintf(fileRUN, '%s%6.4f\n', 'd ', runs.rho);      % density
fprintf(fileRUN, '\n');                             % back to oper



%%%%%%%%%%%%%%%%%%%%%%%%%%%

% enter constraints
con = runs.const;
fprintf(fileRUN, '%s%s%6.4f\n', 'a ', 'c ', con.CL);    % set a to cl
fprintf(fileRUN, '%s%s%6.4f\n', 'b ', 'b ', con.b);     % set b to b
fprintf(fileRUN, '%s%s%6.4f\n', 'r ', 'r ', con.r);     % set r to r
fprintf(fileRUN, '%s%s%6.4f\n', 'p ', 'p ', con.p);     % set p to p
fprintf(fileRUN, '%s%s%6.4f\n', 'y ', 'y ', con.y);     % set y to y
fprintf(fileRUN, '%s%s%6.4f\n', ['d',elevn, ' '], 'pm ', con.pm);     % set y to y


%%%%%%%%%%%%%%%%%%%%%%%%%%%%





% ailerons
% fprintf(fileRUN, '%s%s%6.4f\n', 'd1 ', 'd1 ', rudder.d_ele);   % rudder
% fprintf(fileRUN, '%s%s%6.4f\n', num1, num1, rudder.d_ele);   % rudder
if n_ail > 0
    num1 = 'd1 ';
    fprintf(fileRUN, '%s%s%6.4f\n', num1, num1, wing.d_ail);   % aileron
end
    
% print flaps
for i1 = 1:n_fps            % 1, 2, 3
    indx = n_fps - i1 + 1;  % 3, 2, 1
    num1 = ['d', num2str(n_ail + indx), ' '];
    fprintf(fileRUN, '%s%s%6.4f\n', num1, num1, wing.d_fps(i1));   % flaps
end

% run the analysis
fprintf(fileRUN, '%s\n', 'x');

% save the stability derivatives
if runs.stFlag == 1
    fprintf(fileRUN, '%s %s\n', 'st', [runs.name, '.stabs']);
end

% save the strip forces
if runs.fsFlag == 1
    fprintf(fileRUN, '%s %s\n', 'fs', [runs.name, '.strip']);
    fprintf(fileRUN, '%s %s\n', 'hm', [runs.name, '.hinge']);
end

% quit
fprintf(fileRUN, '\n');                             % back to main
fprintf(fileRUN, 'QUIT\n');                         % all done
fclose(fileRUN);


end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function WritePreamble(fID, mach, sims, refs, offs, cdp, linewrite)
% Function to write the preamble entry of the avl file
% INPUTS:
%   fID         - avl file id
%   mach        - mach number
%   sims        - 3 x 1 vector of Xsym, Ysym, and Zsym flags
%   refs        - 3 x 1 vector of Sref, Cref, Bref values
%   offs        - 3 x 1 vector of Xref, Yref, Zref values
%   cdp         - profile drag coefficient
%   linewrite   - line for spacing

fprintf(fID, '%s\n', linewrite);                    % break line
fprintf(fID, '%s\n', 'TCS');
fprintf(fID, '%s\n', '! Mach');                         % mach number
fprintf(fID, '%1.1f\n', mach);                     % mach number
fprintf(fID, '%s\t%s\t%s\n', '! Xsym', 'Ysym', 'Zsym'); % sym flags
fprintf(fID, '%1.0f %1.0f %1.0f\n', sims(1), ...
    sims(2), sims(3));                % sym flags
fprintf(fID, '%s\t%s\t%s\n', '! Sref', 'Cref', 'Bref'); % ref flags
fprintf(fID, '%5.1f %5.1f %5.1f\n', refs(1), ...
    refs(2), refs(3));
fprintf(fID, '%s\t%s\t%s\n', '! Xref', 'Yref', 'Zref'); % ref flags
fprintf(fID, '%6.2f %6.2f %6.2f\n', offs(1), ...
    offs(2), offs(3));
fprintf(fID, '%s\n', '! CDp');                          % CDp number
fprintf(fID, '%s\n', num2str(cdp));                % CDp number
fprintf(fID, '\n');

end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function WriteSurface(fID, names, chordVort, Cspace, spanVort, Sspace, linewrite)
% Function to write the "SURFACE" entry of the avl file
% INPUTS:
%   fID         - avl file id
%   chordVort   - number of chordwise vortices
%   names       - name of the surface
%   Cspace      - spacing parameter
%   spanVort    - spanwise vortex number
%   Sspace      - spacing method on the wing

fprintf(fID, '%s\n', linewrite);                            % break line
fprintf(fID, '%s\n', 'SURFACE ');                            % surface
fprintf(fID, '%s\n', names);                               % wing 
fprintf(fID, '%s\t%s\t%s\t%s\n', '! Nchordwise', ...        % headers 
    'Cspace', 'Nspanwise', 'Sspace'); 
fprintf(fID, '%d %1.1f %d %1.1f \n', chordVort, ...    % values
    Cspace, spanVort, Sspace);
fprintf(fID, '\n');                                         % newline

end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function WriteSection(fID, x, y, z, chord, ainc, spanVort, Sspace, afil)
% Function to write the "SECTION" entry of the avl file
% INPUTS:
%   fID         - avl file id
%   x           - x entry
%   y           - y entry
%   z           - z entry
%   chord       - chord at section
%   ainc        - angle of attack of section
%   spanVort    - spanwise vortex number
%   Sspace      - spacing method on the wing
%   afil        - airfoil information
%       EXAMPLE:
%       NACA 0015  -> afil = 'NACA 0015' 
%       sd7037.dat -> afil = 'AFIL sd7037.dat' 

% Sspace=0.0; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% change later %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf(fID, '%s\n', 'SECTION');                              % section
fprintf(fID, '%s\t%s\t%s\t%s\t%s\t%s\t%s\n', '! Xle', ...     % header
    'Yle', 'Zle', 'Chord', 'Ainc', 'Nspanwise', 'Sspace'); 
fprintf(fID, '%5.2f %5.2f %5.2f %5.2f %1.1f %d %1.1f\n', ...              % values
    x, y, z, chord, ainc, spanVort, Sspace); 
 

% add airfoil if needed
if isempty(afil) ~= 1
    a = strsplit(afil);     % get airfoil info and throw error if needed
    if length(a) ~= 2
        error(['afil value of "', afil, '" is not in the proper format.']);
    end
    
    fprintf(fID, '%s\n', cell2mat(a(1)));    % airfoil geometry
    fprintf(fID, '%s\n', cell2mat(a(2)));    % airfoil geometry
end

fprintf(fID, '\n');                      % blank line

end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function WriteControl(fID, cname, cgain, xhinge, hingeVec, sgndup)
% Function to write the "CONTROL" entry of the avl file
% INPUTS:
%   fID         - avl file id
%   cname       - name of the control
%   cgain       - control deflect. gain in deg/control variable
%   xhinge      - x/c location of the hinge
%   hingeVec    - 3 x 1 vector of hinge vector rotation
%   sgndup      - sign of deflection 
%       EXAMPLE: 
%       ailerons -> sgndup = -1
%       elevator -> sgndup = +1

fprintf(fID, '%s\n', 'CONTROL');                            % surface
fprintf(fID, '%s\t%s\t%s\t%s\t%s\n', ...
    '! Cname', 'Cgain', 'Xhinge', 'HingeVec', 'SgnDup'); 
fprintf(fID, '%s %1.1f %5.2f %1.1f %1.1f %1.1f %1.1f\n', ...
    cname, cgain, xhinge, hingeVec(1), hingeVec(2), ...
    hingeVec(3), sgndup);
fprintf(fID, '\n');                                         % newline

end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

function WritePreambleMass(fID, Lunit, Munit, Tunit, g, rho)
% Function to write the preamble for the mass files
% INPUTS:
%   fID     - File identifier
%   Lunit   - length unit
%   Munit   - mass unit
%   Tunit   - time unit
%   g       - gravity [Lunit/Tunit^2]
%   rho     - density [Munit/Lunit^3]

% user info
fprintf(fID, '%s \n', '! ');
fprintf(fID, '%s \n', '! TCS AUTO Mass File');
fprintf(fID, '%s \n', '! ');
fprintf(fID, '%s \n', '!xyz is the location of the item CG');
fprintf(fID, '%s \n', '!Ixx, ... are the inertias about the item CG');
fprintf(fID, '%s \n', '! ');
fprintf(fID, '%s \n', '!x back');
fprintf(fID, '%s \n', '!y right');
fprintf(fID, '%s \n', '!z up');
fprintf(fID, '%s \n', '! ');
fprintf(fID, '%s \n', '!x,y,z system must have origin as the AVL input file');
fprintf(fID, '%s \n', '! ');

% alv info
fprintf(fID, '%s %6.4f %s \n', 'Lunit = ', Lunit, ' m');
fprintf(fID, '%s %6.4f %s \n', 'Munit = ', Munit, ' kg');
fprintf(fID, '%s %6.4f %s \n', 'Tunit = ', Tunit, ' s');
fprintf(fID, '%s \n', ' ');
fprintf(fID, '%s %6.4f\n', 'g = ', g);
fprintf(fID, '%s %6.4f\n', 'rho = ', rho);
fprintf(fID, '%s \n', ' ');
fprintf(fID, '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n', ...
    '! Mass', 'x', 'y', 'z', 'Ixx', 'Iyy', 'Izz', 'Ixy', 'Ixz', 'Iyz');
fprintf(fID, '%s \n', '! ');

end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

function WriteMassTable(fID, mass, xcg, ycg, zcg, ...
    IXX, IYY, IZZ, IXY, IXZ, IYZ, comm)
% Function to write a line of the mass table 
% INPUTS:
%   fID     - file identification
%   mass    - mass of the line
%   xcg     - xcg of line
%   ycg     - ycg of line
%   zcg     - zcg of line
%   IXX     - moment of inertia about x
%   IYY     - moment of inertia about y
%   IZZ     - moment of inertia about z
%   IXY     - product of inertia about xy
%   IXZ     - product of inertia about xz
%   IYZ     - product of inertia about YZ
%   comm    - any comments on the line

fprintf(fID, '%7.2f%7.2f%7.2f%7.2f%7.2f%7.2f%7.2f%7.2f%7.2f%7.2f%s\n', ...
    mass, xcg, ycg, zcg, IXX, IYY, IZZ, IXY, IXZ, IYZ, ['   ! ', comm]);

end