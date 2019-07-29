function [alfa, CL, errs, iters] = findStall(design, plotFlag)
% [alfa, CL, errs] = findStall(design) returns the stall angle of 
% attack in degrees for the design structure design. 

% disp stuffs
disp('Finding stall alpha ... ');

% where is avl
alpha1 = 2;
alpha2 = 14;
tol = 5e-3;
exitFlag = 0;

% alpha 1
[CL_low, errs_low] = RunLoop(design, alpha1, 1, 0);

% alpha 2
[CL_high, errs_high] = RunLoop(design, alpha2, 2, 0);

% store values per iteration
iters(1, 1:3) = [alpha1, CL_low, errs_low];
iters(2, 1:3) = [alpha2, CL_high, errs_high];
itx = 3;

% loops on loops
while exitFlag ~= 1
    
    % interpolate to find the stall alpha
    % errs_low is positive
    % errs_high is negative
    if errs_low > 0 && errs_high < 0
        alpha3 = interp1(iters(:, 3), iters(:, 1), 0, 'linear');
    
    else
        alpha3 = interp1(iters(:, 3), iters(:, 1), 0, 'linear', 'extrap');
    
    end
    
    % run
    [CL_new, errs_new] = RunLoop(design, alpha3, itx, 0);
    iters(itx, 1:3) = [alpha3, CL_new, errs_new];
    itx = itx + 1;
    
    % decide if go again
    if abs(errs_new) < tol
        exitFlag = 1;
        alfa = alpha3;
        CL = CL_new;
        errs = errs_new;
    end
        
    % choose new points
    % if errs_new is positive
    % if 1 < 2
    if abs(errs_new) < abs(errs_low)
        errs_low = errs_new;
        
    elseif abs(errs_new) < abs(errs_high)
        errs_high = errs_new;
        
    end
    
end

% plot best case
if plotFlag == 1
    RunLoop(design, alpha3, itx, 1);
end

end


function [CL, errs] = RunLoop(design, alpha, i1, plotFlag)
% [CL, errs] = RunLoop(design, alpha, iteration) returns the cl and 
% error for a given design and alpha run. 

    % set up
    avlLocation = '.\avl.exe';
    nameList = {'alfa'};

    % display info
    disp(['Loop iteration: ', num2str(i1), ...
        ', Alpha [deg]: ', num2str(alpha)]);
    runName = nameList{1};
    
    % set first alpha
    runs = design.runs;
    const = runs.const;
    const.a = alpha;       % set
    runs.name = runName;
    runs.const = const;         % generic constraints
    design.runs = runs;

    % write the files + run
    delete([runName, '.stabs']);
    delete([runName, '.strip']);
    delete([runName, '.mass']);
    delete([runName, '.avl']);
    delete([runName, '.run']);
    [~, ~, runFile] = WriteAVL_D(design);
    fclose('all');
    [status, ~] = dos([avlLocation,' < ', runFile]);
    stabDerivs = ReadStabDerivs([runName, '.stabs']);
    [Cl, ~, ~, ~, ~, yLE] = ReadStrips([runName, '.strip'], 0);
    fclose('all');
    
    % get CL difference
    [Clmax] = getCLmax(design, yLE(:, 1));
    differr = Clmax - Cl(:, 1);
    [m1, indx] = min(differr);

    % return valuables
    CL = stabDerivs.CLtot;
    errs = m1;
    
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