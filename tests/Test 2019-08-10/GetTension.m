function GetTension(wd, filename)
% CDY

% files
fid = fopen(fullfile(wd, filename), 'r');
ttlline = fgetl(fid);
newline = ttlline;  % reset for other stuffs
cntr = 0;
while newline ~= -1
    filenew = fullfile(wd, ['newfile', num2str(cntr), '.txt']);
    fidNEW = fopen(filenew, 'w');
    for i1 = 1:200000*3
        fprintf(fidNEW, [newline, '\n']);
        newline = fgetl(fid);
        newline = fgetl(fid);
        newline = fgetl(fid);
        if newline == -1
            break
        end
    end
    fclose(fidNEW);
    cntr = cntr + 1;    
    
    % add line number to the title line
    
end

end