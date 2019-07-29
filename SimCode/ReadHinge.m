function [hinges] = ReadHinge(filename, varargin)
% [hinges] = ReadHinge(filename) return the hinge moment coefficients as 
% a structure with names of the controls as fields and the moment values 
% as elements. 
% [hinges] = ReadHinge(filename, design) returns the structure of hinge
% moments as elements for the field names of the controls. design must be
% either a design structure used to generate the avl file or a matrix file
% of a previous run. 

% check varargin
rho = NaN;
vel = NaN;
q1 = NaN;
if isempty(varargin) ~= 1   
    design1 = varargin{1};        % overall structure
    
    % if matrix file
    if ischar(design1) == 1
        s1 = load(design1);
        designm = s1.design;
        wingS = designm.wing;         % wing structure in design structure
        rudderS = designm.rudder;     % rudder structure in design structure
        runsS = designm.runs;
        rho = runsS.rho;     % density
        vel = runsS.vel;     % velocity
        
    elseif isstruct(design1) == 1
        wingS = design1.wing;         % wing structure in design structure
        rudderS = design1.rudder;     % rudder structure in design structure
        runsS = design1.runs;
        rho = runsS.rho;     % density
        vel = runsS.vel;     % velocity
        
    else
        error('Input design is not of structure or matrix file format.');
        
    end
    q1 = 0.5*rho*vel*vel;       % calculate dynamic pressure
    
end


% read values and return
fID = fopen(filename);
newLine = fgetl(fID);
exitFlag = 0;
while exitFlag ~= -1
    
    a = newLine(1:10);
    switch a
        case ' (referred'
            b = strsplit(newLine);
            
            for i1 = 1:length(b)
                switch b{i1}
                    case 'Sref'
                        hinges.Sref = str2double(b{i1 + 2});
                        Sref = hinges.Sref;
                        
                    case 'Cref'
                        c = b{i1 + 2};
                        hinges.Cref = str2double(c(1:end - 1));
                        Cref = hinges.Cref;
                        
                end
            end
            
            % apply multiplication values of 1 if no file info present
            if isnan(q1) == 1
                Sref = 1;
                Cref = 1;
                q1 = 1;
            end
            
        case ' Control  '
            newLine = fgetl(fID);
            newLine = fgetl(fID);
            while strcmp(newLine(1:6), ' -----') ~= 1
                b = strsplit(newLine);
                eval(['hinges.', b{2}, ' = ', b{3}, '*Sref*Cref*q1;']);
                newLine = fgetl(fID);
            end
            
    end
    
    newLine = fgetl(fID);
    if isempty(newLine) == 1
        newLine = fgetl(fID);
    elseif newLine == -1 
        exitFlag = -1;
    end
    
end

end