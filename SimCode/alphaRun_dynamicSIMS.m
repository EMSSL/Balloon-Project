function [CL, CD, xnp, yL] = alphaRun_dynamicSIMS(design, alpha)
% [CL, errs] = alphaRun(design, alpha) returns the 
% stall angle of attack in degrees for the design structure design. 

% setup
CL = NaN(length(alpha), 1);
CD = NaN(length(alpha), 1);
xnp = NaN(length(alpha), 1);
yL = NaN(length(alpha), 1);


for i1 = 1:length(alpha)
    
    % run the analysis
    disp(['Alpha = ', num2str(alpha(i1)), ' deg']);
    exten = sprintf('_%d', i1);
    [CLi, CDi, Xnpi, yli] = RunLoop(design, alpha(i1), 0, exten);
    CL(i1) = CLi;
    CD(i1) = CDi;
    xnp(i1) = Xnpi;
    yL(i1) = yli;
    
end


end


function [CL, CD, Xnp, yl] = RunLoop(design, alpha, plotFlag, exten)
% [CL, errs] = RunLoop(design, alpha, plotFlag) returns the cl and 
% error for a given design and alpha run. 

    % set up
    avlLocation = '.\avl.exe';
    % nameList = {'subsc'};
    % runName = nameList{1};
    
    % set first alpha
    runs = design.runs;
    const = runs.const;
    const.a = alpha;       % set
    runs.name = [runs.name, exten];
    runs.const = const;         % generic constraints
    design.runs = runs;

    % write the files + run
    fclose('all');
    delete([runName, '.stabs']);
    delete([runName, '.strip']);
    delete([runName, '.mass']);
    delete([runName, '.avl']);
    delete([runName, '.run']);
    [~, ~, runFile] = WriteAVL_E(design);   % _E is no trim, _D is trim
    fclose('all');
    
    % run the analysis
    [status, ~] = dos([avlLocation,' < ', runFile]);
    
%     % use for trouble shooting
%     [status, ~] = dos([avlLocation,' < ', runFile], '-echo');
    
    % read in info
    [~, ~, ~, ~, ~, ~, ~, ~, ~, yl, ~] = ReadStrips([runName, '.strip'], 1, design);
    % keyboard;
    stabDerivs = ReadStabDerivs_revA([runName, '.stabs']);
    fclose('all');
    
    keyboard

    % return valuables
    CL = stabDerivs.CLtot;
    CD = stabDerivs.CDtot;
    Xnp = stabDerivs.Xnp;
    
    
    
    
    % plot if asked
    if plotFlag == 1
        figure
        hold on
        plot(yLE(:, 1), Clmax, 'r-')
        plot(yLE(:, 1), Cl(:, 1))
        xlabel('Span position [m]')
        ylabel('Sectional lift coefficient')
        grid on
        grid minor
        title(['$C_{L}$ = ', num2str(CL), ...
            ', $\alpha_{Stall}$ = ', num2str(alpha), ' [deg]']);
        legend({'$C_{l_{max}}$', '$C_{l_{sail}}$'}, ...
            'interpreter', 'latex', 'location', 'east');
        
        wing = design.wing;
        spn = wing.span;
        set(gca, 'XTick', -1.6:0.4:0);
        xlim([-spn, 0]);
        set(gca, 'YTick', 0:0.4:1.6);
        ylim([0, 1.6])
        
    end

end