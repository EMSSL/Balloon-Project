function tempFunc(filename)

% preamble
numc = 2000;
Cl = NaN(numc, numc);
Cd = NaN(numc, numc);
ai = NaN(numc, numc);
cmc4 = NaN(numc, numc);
chord = NaN(numc, numc);
yLE = NaN(numc, numc);
nmbr = NaN(numc, numc);

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
    newLine = fgetl(fID)

end

keyboard

end