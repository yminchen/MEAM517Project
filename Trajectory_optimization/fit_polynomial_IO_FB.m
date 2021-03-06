clear;
close all
% load('Nomial_trajectories/Traj2/Data1Step_30-Nov-2018_100nodes.mat');  
load('Nomial_trajectories/Traj3/Data1Step_04-Dec-2018.mat');  
% load('Nomial_trajectories/Traj4/Data1Step_04-Dec-2018_lessFootClearance.mat');  

addpath('../Functions/');   % for approx_leg_angle()

%% Extract data
len = size(x1(:,1),1);
q_data = x1;
t = [0:t1/(len-1):t1]';
colheaders = {'x', 'y', 'phi', 'alphaR', 'betaR', 'alphaL', 'betaL'};

%% Parameters
chosenColumnList = [4,5,6,7]; % Choose the column of data you want to fit.
polyOrder = 12;               % Set the order of the Polynomial
isCheckFitting = true;       % Whether plot the fitting polynomial or not
isIOFeedbackLinearization = false;

%% Pick the monotonically increasing function theta
% You can choose time or the leg angle as the function (or x position, etc)

% theta = t;                         % use time as the phase variable
theta = approx_leg_angle(q_data(:,3), q_data(:,4), q_data(:,5)); % approximated leg angle

%% Symbolically compute input output feedback linearization 
if isIOFeedbackLinearization
    % Declare symbolic variables and calculate EoM
    addpath('../Function_generator/EOM_minimal_coord');
    disp('Creating EoM...')
    declare_syms_and_calc_EoM_minCoord;
    disp('Finished EoM.')
    B=zeros(7,4); B(4:7,:)=eye(4);
    % The equations of motion given by these functions are:   
    %   M * ddq = fCG + B*u;
    % Note that it has to be minimal coordinates. (no constraint forces in the EoM)
    % Otherwise, you cannot get u by IO FB linearization (at least I don't know how)

    % theta
    theta_of_q = approx_leg_angle(phi, alphaStance, betaStance);

    % output y = y_first_term - h_d
    y_first_term = [ alphaStance;
                     betaStance;
                     alphaSwing;
                     betaSwing];
end

%% Fit polynomials to the data

coeff_all = [];

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
%     display(coeff);
    
    coeff_all = [coeff_all; coeff'];
    
    %% Check how good the fitting is
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
        xlabel('theta (rad)');
        ylabel(colheaders{chosenColumn});
        hold on

        %% Plot the fitting function over theta (compare it with the original data)
        LSFuncData = zeros(len,1);
        for i = 1:len
            LSFuncData(i) = LSFunc((theta(i,1)-theta(1,1))/(theta(end,1)-theta(1,1)));
        end

        plot(theta(:,1),LSFuncData);
        
        legend('Optimization data','Polynomial')
        title('Data fitting')
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

end


%% IO FB linearization 
if isIOFeedbackLinearization
    
%% Symbolically compute input output feedback linearization 
% 
% % Choose which joint to be the output
% chosenColumnForOutput = chosenColumnList;
% 
% % Get the coeffcients coresponding to the outputs that we choose
% chosenCoeffRow = [];
% for i = 1:numel(chosenColumnForOutput)
%     chosenCoeffRow = [chosenCoeffRow, find(chosenColumnList==chosenColumnForOutput(i),1)];
% end
% 
% % Construct h_d(q), where theta is the monotonically increase function (could be leg angle)
% h_d = zeros(numel(chosenCoeffRow),1);
% for i = 0:polyOrder
%     h_d = h_d + coeff_all(chosenCoeffRow,i+1)*(theta_of_q^i);
% end
% 
% % Construct output y
% y = y_first_term - h_d;
% 
% % IO feedback linearization
% yDot = jacobian(y,qm)*dqm;
% % The second derivative involves inverse of the mass matrix, so we are not
% % going to get it symbolically here. Instead, we will only get parts of the
% % equation symbolically, and then solve it numerically when using it. 
% % Equation:
% % yDDot = d(yDot)/dx * dx/dt
% %        = d(yDot)/dq * dq/dt + d(yDot)/ddq * ddq/dt
% %        = d(yDot)/dq * dq/dt + d(yDot)/ddq * [(M^-1)*(fCG + B*u)]
% %        = L_f_2_h + L_g_L_f_h * u
% % If we let   
% %     d_yDot_dq_times_dqdt = d(yDot)/dq * dq/dt
% %     d_yDot_ddq           = d(yDot)/ddq
% % then we can rewrite yDDot as
% %     yDDot     = d_yDot_dq_times_dqdt + d_yDot_ddq * (M^-1)*(fCG + B*u)
% % and we also have 
% %     L_f_2_h   = d_yDot_dq_times_dqdt + d_yDot_ddq * (M^-1) * fCG
% %     L_g_L_f_h =                        d_yDot_ddq * (M^-1) * B 
% % 
% % That is, we can derive L_f_2_h and L_g_L_f_h in terms of 
% % d_yDot_dq_times_dqdt, d_yDot_ddq, M, fCG and B.
% 
% d_yDot_dq_times_dqdt = jacobian(yDot,qm)*dqm; 
% d_yDot_ddq = jacobian(yDot,dqm);
% 
% if ~exist('Auto_generated','dir')
%     mkdir('Auto_generated');
% end
% 
% if numel(chosenCoeffRow)>1
%     % also need to create y and ydot
%     matlabFunction(y,'file',['Auto_generated/y_output_','vector'],'vars',{[qm;dqm],sysParam_minCoord});
%     matlabFunction(yDot,'file',['Auto_generated/dy_output_','vector'],'vars',{[qm;dqm],sysParam_minCoord});
%     matlabFunction(d_yDot_dq_times_dqdt,'file',['Auto_generated/d_yDot_dq_times_dqdt_','vector'],'vars',{[qm;dqm],sysParam_minCoord});
%     matlabFunction(d_yDot_ddq,'file',['Auto_generated/d_yDot_ddq_','vector'],'vars',{[qm;dqm],sysParam_minCoord});
% else 
%     matlabFunction(y,'file',['Auto_generated/y_output_',colheaders{chosenCoeffRow}],'vars',{[qm;dqm],sysParam_minCoord});
%     matlabFunction(yDot,'file',['Auto_generated/dy_output_',colheaders{chosenCoeffRow}],'vars',{[qm;dqm],sysParam_minCoord});
%     matlabFunction(d_yDot_dq_times_dqdt,'file',['Auto_generated/d_yDot_dq_times_dqdt_',colheaders{chosenCoeffRow}],'vars',{[qm;dqm],sysParam_minCoord});
%     matlabFunction(d_yDot_ddq,'file',['Auto_generated/d_yDot_ddq_',colheaders{chosenCoeffRow}],'vars',{[qm;dqm],sysParam_minCoord});
% end

%% Symbolically compute input output feedback linearization V2 (leave theta intact)
% This version leave theta in the polynomial so that we can restrict it
% within theta_min and theta_max when we use the function (outside the range the polynomial is very inaccurate)

% Choose which joint to be the output
chosenColumnForOutput = chosenColumnList;

% Get the coeffcients coresponding to the outputs that we choose
chosenCoeffRow = [];
for i = 1:numel(chosenColumnForOutput)
    chosenCoeffRow = [chosenCoeffRow, find(chosenColumnList==chosenColumnForOutput(i),1)];
end

syms theta real

% Construct h_d(q), where theta is the monotonically increase function (could be leg angle)
h_d = zeros(numel(chosenCoeffRow),1);
for i = 0:polyOrder
    h_d = h_d + coeff_all(chosenCoeffRow,i+1)*(theta^i);
end

% Construct output y
y = y_first_term - h_d;

% IO feedback linearization (derivation in your notes)
% First derivative
yDot = jacobian(y,qm)*dqm + jacobian(y,theta)*jacobian(theta_of_q,qm)*dqm;

% The second derivative involves inverse of the mass matrix, so we are not
% going to get it symbolically here. Instead, we will only get parts of the
% equation symbolically, and then solve it numerically when using it. 
% The derivation is in your notes.
d_yDot_dq_times_dqdt = jacobian(yDot,qm)*dqm; 
d_yDot_dtheta_times_dthetadt = jacobian(yDot,theta)*jacobian(theta_of_q,qm)*dqm; 
d_yDot_ddq = jacobian(yDot,dqm);

IO_FB_term1 = d_yDot_dq_times_dqdt + d_yDot_dtheta_times_dthetadt;
IO_FB_term1 = simplify(IO_FB_term1);

if ~exist('Auto_generated','dir')
    mkdir('Auto_generated');
end

if numel(chosenCoeffRow)>1
    % also need to create y and ydot
    matlabFunction(y,'file',['Auto_generated/y_output_theta_','vector'],'vars',{[qm;dqm],theta,sysParam_minCoord});
    matlabFunction(yDot,'file',['Auto_generated/dy_output_theta_','vector'],'vars',{[qm;dqm],theta,sysParam_minCoord});
    matlabFunction(IO_FB_term1,'file',['Auto_generated/IO_FB_term1_theta_','vector'],'vars',{[qm;dqm],theta,sysParam_minCoord});
    matlabFunction(d_yDot_ddq,'file',['Auto_generated/d_yDot_ddq_theta_','vector'],'vars',{[qm;dqm],theta,sysParam_minCoord});
else 
    matlabFunction(y,'file',['Auto_generated/y_output_theta_',colheaders{chosenCoeffRow}],'vars',{[qm;dqm],theta,sysParam_minCoord});
    matlabFunction(yDot,'file',['Auto_generated/dy_output_theta_',colheaders{chosenCoeffRow}],'vars',{[qm;dqm],theta,sysParam_minCoord});
    matlabFunction(IO_FB_term1,'file',['Auto_generated/IO_FB_term1_theta_',colheaders{chosenCoeffRow}],'vars',{[qm;dqm],theta,sysParam_minCoord});
    matlabFunction(d_yDot_ddq,'file',['Auto_generated/d_yDot_ddq_theta_',colheaders{chosenCoeffRow}],'vars',{[qm;dqm],theta,sysParam_minCoord});
end

end



