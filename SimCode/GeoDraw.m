function GeoDraw(design)
% GeoDraw(design) plots the geoemtry of the sail (flaps, ailerons, and all)
% on the current figure open. 

% plot stuffs
LineWidth = 1.5;
clrs = [0         0.4470    0.7410
        0.8500    0.3250    0.0980
        0.9290    0.6940    0.1250
        0.4940    0.1840    0.5560
        0.4660    0.6740    0.1880
        0.3010    0.7450    0.9330
        0.6350    0.0780    0.1840];
cntr = 1;
lege1{cntr} = 'Sail';

% inputs
wing = design.wing;         % wing structure in design structure
rudder = design.rudder;     % rudder structure in design structure
runs = design.runs;         % runs structure in design structure

% draw wing geoemtry
xle = wing.xle;
yle = wing.yle;
span = wing.span;
chord = wing.chord;
aw = [0, 0, chord, chord, 0] + xle;
bw = [0, span, span, 0, 0] + yle;

xle1 = rudder.xle;
yle1 = rudder.yle;
span1 = rudder.span;
chord1 = rudder.chord;
ar = [0, 0, chord1, chord1, 0] + xle1;
br = [0, span1, span1, 0, 0] + yle1;

hold on
xv = [aw, NaN, ar];
yv = [bw, NaN, br];
plot(xv, yv, 'k-', 'LineWidth', LineWidth);
set(gca, 'YDir', 'reverse');


% % draw the rudder
% hold on
% xv = [0, 0, chord, chord, 0] + xle;
% yv = [0, span, span, 0, 0] + yle;
% plot(xv, yv, 'k-', 'LineWidth', LineWidth);
% % set(gcf, 'Position', [870   557   716   554]);

% draw ailerons (if present)
if wing.n_ail == 1
    
    xb = wing.xbc_ail*chord;
    yb = wing.e_ail;
    
    xv = [xb, xb, chord, chord, xb] + xle;
    yv = [yb, wing.s_ail, wing.s_ail, yb, yb] + yle;
    fill(xv, yv, clrs(cntr, :));
    cntr = cntr + 1;
    lege1{cntr} = 'Aileron';
       
end

% draw flaps (if present)
if isempty(wing.n_fps) ~= 1
        
    for i1 = 1:wing.n_fps
        indx = wing.n_fps - i1 + 1;
        xb = wing.xbc_fps*chord;
        yb = wing.e_fps(indx);
    
        xv = [xb, xb, chord, chord, xb] + xle;
        yv = [yb, wing.s_fps(indx), wing.s_fps(indx), yb, yb] + yle;
        fill(xv, yv, clrs(cntr, :));
        cntr = cntr + 1;
        lege1{cntr} = ['Flap ', num2str(indx)];
        
    end
end

% draw rudder geoemtry
xle = rudder.xle;
yle = rudder.yle;
span = rudder.span;
chord = rudder.chord;

% draw elevator (if present)
xb = rudder.xbc_ele*chord;

xv = [xb, xb, chord, chord, xb] + xle;
yv = [0, span, span, 0, 0] + yle;
fill(xv, yv, clrs(cntr, :));
cntr = cntr + 1;
lege1{cntr} = 'Elevator';

legend(lege1, 'interpreter', 'latex', 'location', 'southeast')
grid on
grid minor

end