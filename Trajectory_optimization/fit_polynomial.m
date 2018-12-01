clear;
close all
load('Nomial_trajectories/Traj2/Data1Step_30-Nov-2018_100nodes.mat');  

%% Extract data
len = size(x1(:,1),1);
q_data = x1;
t = [0:t1/(len-1):t1]';
colheaders = {'x', 'y', 'phi', 'alphaR', 'betaR', 'alphaL', 'betaL'};

%% Parameters
chosenColumnList = [4,5,6,7]; % Choose the column of data you want to fit.
polyOrder = 12;               % Set the order of the Polynomial

%% Pick the monotonically increasing function theta
% You can choose time or the leg angle as the function (or x position, etc)

% theta = t;                         % use time as the phase variable
theta = -(q_data(:,3)+q_data(:,4)+q_data(:,5)/2);   % approximated leg angle

%% Symbolically compute input output feedback linearization 
% Declare symbolic variables and calculate EoM
addpath('../Function_generator/EOM');
disp('Creating EoM...')
declare_syms_and_calc_EoM;
disp('Finished EoM.')
B=zeros(7,4); B(4:7,:)=eye(4);
% The equations of motion are given with these functions as:   
% M * ddq = fCG + B*u;

% theta
theta_symbolic = -(phi + alphaR + betaR/2);


%% Fit polynomials to the data

lenChosenColumnList = numel(chosenColumnList);
for chosenColumnListID = 1:lenChosenColumnList  % loop through every column chosen
    chosenColumn = chosenColumnList(chosenColumnListID);

    %% Choose the data that we want to fit
    chosenData = q_data(1:end,chosenColumn);

    %% Fitting (least square)
    % A*coeff = chosenData

    A = zeros(size(theta,1),polyOrder+1);
    for i = 0:polyOrder
        A(:,i+1) = theta.^i;
    end

    coeff = (A'*A)\(A'*chosenData);
    display(coeff);

    %% Check how good the fitting is
    isCheckFitting = false;
    if isCheckFitting
        %% Create function 
        syms z real

        LSCurve = 0;
        for i = 0:polyOrder
            LSCurve = LSCurve + (( z*(theta(end,1)-theta(1,1))+theta(1,1) )^i)*coeff(i+1);
        end
        LSCurve = simplify(expand(LSCurve));
        disp(LSCurve);

        % simplify the polynomial coefficient
        isSimplifyCoefficient = true;
        if isSimplifyCoefficient
            coeff_z = sym2poly(LSCurve);
            coeff_z = coeff_z(end:-1:1);
            precision_digit = 9; % the number will have precision_digit+1 number of non-zero digit
            for i = 1:(polyOrder+1)
                power = floor(log10(abs(coeff_z(i))));
                coeff_z(i) = round(coeff_z(i), -(power-precision_digit));
            end
            % construct the polynomial with less precise coefficients 
            LSCurve = 0;
            for i = 0:polyOrder
                LSCurve = LSCurve + (z^i)*coeff_z(i+1);
            end
            disp(LSCurve);
        end

        if ~exist('Auto_generated','dir')
            mkdir('Auto_generated');
        end
        matlabFunction(LSCurve,'file','Auto_generated/LSFunc','vars',z);
        addpath('Auto_generated');

        %% Plot original data over the theta variable (un-normalized phase variable)
        figure;
        plot(theta,q_data(:,chosenColumn));
        xlabel('theta');
        ylabel(colheaders{chosenColumn});
        hold on

        %% Plot the fitting function over theta (compare it with the original data)
        LSFuncData = zeros(len,1);
        for i = 1:len
            LSFuncData(i) = LSFunc((theta(i,1)-theta(1,1))/(theta(end,1)-theta(1,1)));
        end

        plot(theta(:,1),LSFuncData);
        hold off    

        %% Plot the fitting function over phase (from 0 to 1)
        isPlotFunctionOverPhase = false;
        if isPlotFunctionOverPhase
            LSFuncData = zeros(len,1);
            phase = 0:1/(len-1):1;
            for i = 1:len
                LSFuncData(i) = LSFunc(phase(i));
            end
            figure;
            plot(phase,LSFuncData);
            xlabel('phase')
            ylabel(colheaders{chosenColumn});
        end
    end

    %% Symbolically compute input output feedback linearization 

    % Construct h_d(theta), where theta is the monotonically increase function (could be leg angle)
    h_d = 0;
    for i = 0:polyOrder
        h_d = h_d + (theta_symbolic^i)*coeff(i+1);
    end

    % IO feedback linearization
    hdDot = jacobian(h_d,q)*vq;
    % The second derivative involves inverse of the mass matrix, so we are not
    % going to get it symbolically here. Instead, we will only get parts of the
    % equation symbolically, and then solve it numerically when using it. 
    % Equation:
    % hdDDot = d(hdDot)/dx * dx/dt
    %        = d(hdDot)/dq * dq/dt + d(hdDot)/ddq * ddq/dt
    %        = d(hdDot)/dq * dq/dt + d(hdDot)/ddq * [(M^-1)*(fCG + B*u)]
    %        = L_f_2_h + L_g_L_f_h * u
    % If we let   
    %     d_hdDot_dq_times_dqdt = d(hdDot)/dq * dq/dt
    %     d_hdDot_ddq           = d(hdDot)/ddq
    % then we can rewrite hdDDot as
    %     hdDDot    = d_hdDot_dq_times_dqdt + d_hdDot_ddq * (M^-1)*(fCG + B*u)
    % and we also have 
    %     L_f_2_h   = d_hdDot_dq_times_dqdt + d_hdDot_ddq * (M^-1) * fCG
    %     L_g_L_f_h =                         d_hdDot_ddq * (M^-1) * B 
    % 
    % That is, we can derive L_f_2_h and L_g_L_f_h in terms of 
    % d_hdDot_dq_times_dqdt, d_hdDot_ddq, M, fCG and B.

    d_hdDot_dq_times_dqdt = jacobian(hdDot,q)*vq; 
    d_hdDot_ddq = jacobian(hdDot,vq);

    if ~exist('Auto_generated','dir')
        mkdir('Auto_generated');
    end
    matlabFunction(d_hdDot_dq_times_dqdt,'file',['Auto_generated/d_hdDot_dq_times_dqdt_',colheaders{chosenColumn}],'vars',{[q;vq],param});
    matlabFunction(d_hdDot_ddq,'file',['Auto_generated/d_hdDot_ddq_',colheaders{chosenColumn}],'vars',{[q;vq],param});
    
end


