function [ms, volts, tens] = ReadTension(filename)

fid = fopen(filename, 'r');
ttlline = fgetl(fid);
newline = fgetl(fid);
linenum = 200000*3; % copy+paste from GetTension.m max iteration limit
ms = NaN(linenum, 1);
volts = ms;
tens = ms;
cntr = 1;
while newline ~= -1
    c1 = strsplit(newline, '  ');
    for j1 = 1:3
        ms(cntr) = str2double(c1{1});
        volts(cntr) = str2double(c1{2});
        tens(cntr) = str2double(c1{3});
    end
    newline = fgetl(fid);    
    cntr = cntr + 1;
    if cntr > linenum
        warning('Max line number exceeded.');
        break
    end
end

end