% [stabDerivs] = ReadStabDerivs(filename) returns a structure of stability
% derivatives written to filename by AVL. 
%
% INPUTS:
%   filename    = [string] filename of stability derivatives file
%
% OUTPUTS:
%   stabDerivs  = structure with the stability derivatives (and other
%                   helpful information)
%
% EXAMPLE: extract the stability derivatives from vanilla.stabs
%
% [stabDerivs] = ReadStabDerivs(vanilla.stabs)
%
% stabDerivs = 
% 
%   struct with fields:
% 
%          e: 0.9681
%        CLa: 5.0222
%        CLb: 0
%        CYa: 0
%        CYb: -0.2680
%        Cla: 0
%        Clb: -0.1956
%        Cma: -0.8081
%        Cmb: 0
%        Cna: 0
%        Cnb: 0.0343
%        CLp: 0
%        CLq: 6.7951
%        CLr: 0
%        CYp: -0.3153
%        CYq: 0
%        CYr: 0.1916
%        Clp: -0.5674
%        Clq: 0
%        Clr: 0.1225
%        Cmp: 0
%        Cmq: -21.5664
%        Cmr: 0
%        Cnp: -0.0452
%        Cnq: 0
%        Cnr: -0.0449
%       CLd1: 0.0438
%       CLd2: 0
%       CLd3: 0.0069
%       CLd4: 0
%       CYd1: 0
%       CYd2: 0.0063
%       CYd3: 0
%       CYd4: -0.0021
%       Cld1: 0
%       Cld2: 0.0090
%       Cld3: 0
%       Cld4: -7.8000e-05
%       Cmd1: 0.0097
%       Cmd2: 0
%       Cmd3: -0.0281
%       Cmd4: 0
%       Cnd1: 0
%       Cnd2: 5.2000e-05
%       Cnd3: 0
%       Cnd4: 8.1700e-04
%     CDffd1: 0.0011
%     CDffd2: 0
%     CDffd3: 8.8000e-05
%     CDffd4: 0
%        ed1: -4.0900e-04
%        ed2: 0
%        ed3: 0.0155
%        ed4: 0
%        Xnp: 0.6448

function [stabDerivs] = ReadStabDerivs(filename)
% Written by Christopher D. Yoder on 9/1/2017 to read avl output file of
% stability derivatives.


% error handling
if ischar(filename) ~= 1
    error('Filename is not a character array.');
end

% do what you came to do
fID = fopen(filename);  % open the file
newLine = fgetl(fID);   % get the first line
exitFlag = 0;           % preallocate

% read
while exitFlag ~= 1     % while not finished yet
    
    % check if skip
    if isempty(newLine) ~= 1      % if not blank
        
        % check type of line
        switch newLine(1:6)         % get the first 6 characters
            
            % get Sref, Cref, Bref
            case '  Sref'
                a = strsplit(newLine);      % split string by delimiter
                for i1 = 1:length(a)        % for all, find e and save it
                    if strcmp(cell2mat(a(i1)), '=') == 1
                        eval(['stabDerivs.', a{i1 - 1}, ' = ', a{i1 + 1}, ';']);
                    end
                end
                
                
            % get Xref, Yref, Zref
            case '  Xref'
                a = strsplit(newLine);      % split string by delimiter
                for i1 = 1:length(a)        % for all, find e and save it
                    b1 = strsplit(a{i1}, '=');
                    if length(b1) > 1 && strcmp(a{i1}, '=') ~= 1
                        eval(['stabDerivs.', a{i1 - 1}, ' = ', b1{end}, ';']);
                    elseif strcmp(cell2mat(a(i1)), '=') == 1
                        eval(['stabDerivs.', a{i1 - 1}, ' = ', a{i1 + 1}, ';']);
                    end
                end
            
            % get alpha 
            case '  Alph'
                a = strsplit(newLine);      % split string by delimiter
                for i1 = 1:length(a)        % for all, find e and save it
                    if strcmp(cell2mat(a(i1)), 'Alpha') == 1
                        stabDerivs.Alpha = str2double(cell2mat(a(i1 + 2)));
                        break
                    end
                end
                
            % get beta
            case '  Beta'
                a = strsplit(newLine);      % split string by delimiter
                for i1 = 1:length(a)        % for all, find e and save it
                    if strcmp(cell2mat(a(i1)), 'Beta') == 1
                        stabDerivs.Beta = str2double(cell2mat(a(i1 + 2)));
                        break
                    end
                end
            
            % get Cl'mom
            case '  CXto'
                a = strsplit(newLine);      % split string by delimiter
                for i1 = 1:length(a)        % for all, find e and save it
                    if strcmp(cell2mat(a(i1)), 'Cltot') == 1
                        stabDerivs.Clptot = str2double(cell2mat(a(i1 + 5)));
                        break
                    end
                end
                
            % get Cmtot
            case '  CYto'
                a = strsplit(newLine);      % split string by delimiter
                for i1 = 1:length(a)        % for all, find e and save it
                    if strcmp(cell2mat(a(i1)), 'Cmtot') == 1
                        stabDerivs.Cmtot = str2double(cell2mat(a(i1 + 2)));
                        break
                    end
                end

            % get Cntot
            case '  CZto'
                a = strsplit(newLine);      % split string by delimiter
                for i1 = 1:length(a)        % for all, find e and save it
                    if strcmp(cell2mat(a(i1)), 'Cntot') == 1
                        stabDerivs.Cnptot = str2double(cell2mat(a(i1 + 5)));
                        break
                    end
                end                
                
            % get CL TOTAL
            case '  CLto'
                a = strsplit(newLine);      % split string by delimiter
                for i1 = 1:length(a)        % for all, find e and save it
                    if strcmp(cell2mat(a(i1)), 'CLtot') == 1
                        stabDerivs.CLtot = str2double(cell2mat(a(i1 + 2)));
                        break
                    end
                end                
                
            % get CD TOTAL
            case '  CDto'
                a = strsplit(newLine);      % split string by delimiter
                for i1 = 1:length(a)        % for all, find e and save it
                    if strcmp(cell2mat(a(i1)), 'CDtot') == 1
                        stabDerivs.CDtot = str2double(cell2mat(a(i1 + 2)));
                        break
                    end
                end    
                
            % get CDvis
            case '  CDvi'
                a = strsplit(newLine);      % split string by delimiter
                for i1 = 1:length(a)        % for all, find e and save it
                    if strcmp(cell2mat(a(i1)), 'CDvis') == 1
                        stabDerivs.CDvis = str2double(cell2mat(a(i1 + 2)));
                        break
                    end
                end 
                
            % get e for Oswalds
            case '  CYff'
                a = strsplit(newLine);      % split string by delimiter
                for i1 = 1:length(a)        % for all, find e and save it
                    if strcmp(cell2mat(a(i1)), 'e') == 1
                        stabDerivs.e = str2double(cell2mat(a(i1 + 2)));
                        break
                    elseif strcmp(cell2mat(a(i1)), 'CYff') == 1
                        stabDerivs.CYff = str2double(cell2mat(a(i1 + 2)));
                    end
                end
                
            % get aileron
            case '   ail'
                a = strsplit(newLine);      % split string by delimiter
                for i1 = 1:length(a)        % for all, find e and save it
                    if strcmp(cell2mat(a(i1)), 'ailer') == 1
                        stabDerivs.ailer = str2double(cell2mat(a(i1 + 2)));
                        break
                    end
                end 
                
            % get flap
            case '   fla'
                a = strsplit(newLine);      % split string by delimiter
                for i1 = 1:length(a)        % for all, find e and save it
                    if strcmp(cell2mat(a(i1)), 'flap3') == 1
                        stabDerivs.flap3 = str2double(cell2mat(a(i1 + 2)));
                        break
                    elseif strcmp(cell2mat(a(i1)), 'flap2') == 1
                        stabDerivs.flap2 = str2double(cell2mat(a(i1 + 2)));
                        break
                    elseif strcmp(cell2mat(a(i1)), 'flap1') == 1
                        stabDerivs.flap1 = str2double(cell2mat(a(i1 + 2)));
                        break
                    end
                end 
            
            % get e for Oswalds
            case '   ELE'
                a = strsplit(newLine);      % split string by delimiter
                for i1 = 1:length(a)        % for all, find e and save it
                    if strcmp(cell2mat(a(i1)), 'ELEV') == 1
                        stabDerivs.elev = str2double(cell2mat(a(i1 + 2)));
                        break
                    end
                end
                
                
                
                
                
            % get the stability axis derivatives
            case  ' Stabi'
                           
                % find and save stab derivs
                while exitFlag ~= 1
                    newLine = fgetl(fID);
                    
                    if length(newLine) > 5
                        a = strsplit(newLine);      % split string by delimiter
                        
                        
                        % handle neutral point entry
                        if strcmp(cell2mat(a(2)), 'Neutral') == 1
                            tf = cell2mat(a(4));    % field string
                            tv = cell2mat(a(6));    % field value
                            eval(['stabDerivs.', tf, ' = ', tv, ';'])  % save to fields
                            exitFlag = 1;
                        else
                        
                            % handle check for stability derivatives
                            for i2 = 1:length(a)                 % for all, find stab derivs
                                if strcmp(cell2mat(a(i2)), '=') == 1
                                    tf = cell2mat(a(i2 - 1));    % field string
                                    tv = cell2mat(a(i2 + 1));    % field value
                                    eval(['stabDerivs.', tf, ' = ', tv, ';'])  % save to fields
                                end
                            end
                        end
                    end
                    
                    % keyboard
                end
                
            otherwise
                newLine = fgetl(fID);
             
        end
                
                    
    end
    
    % new line and repeat
    newLine = fgetl(fID);
    
end          

% change names to solve merge issues
stabDerivs.CLde = stabDerivs.CLd1;
stabDerivs.CYde = stabDerivs.CYd1;
stabDerivs.CDffde = stabDerivs.CDffd1;
stabDerivs.ede = stabDerivs.ed1;
stabDerivs.Clde = stabDerivs.Cld1;
stabDerivs.Cmde = stabDerivs.Cmd1;
stabDerivs.Cnde = stabDerivs.Cnd1;


end