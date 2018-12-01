clear;
close all
load('Nomial_trajectories/Traj2/Response30-Nov-2018_100nodes.mat');  

%% Extract data
q = response.states{1}(1:7,:)';
t = response.time{1}';
colheaders = response.stateNames(1:7);
len = size(q(:,1),1);

%% Parameters
columnChoice = 4; % Choose the column of data you want to fit.
polyOrder = 12;    % Set the order of the Polynomial

%% Pick the monotonically increasing function theta
% You can choose time or the leg angle as the function (or x position, etc)

% theta = t;                         % use time as the phase variable
theta = -(q(:,3)+q(:,4)+q(:,5)/2);   % approximated leg angle

%% Plot original data over the theta variable (un-normalized phase variable)
figure;
plot(theta,q(:,columnChoice));
xlabel('theta');
ylabel(colheaders{columnChoice});
hold on

%% Choose the data that we want to fit
chosenData = q(1:end,columnChoice);

%% Fitting
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
        ylabel(colheaders{columnChoice});
    end
end

%% Symbolically compute input output feedback linearization 
syms theta real

% Construct h_d(theta), where theta is the monotonically increase function (could be leg angle)
h_d = 0;
for i = 0:polyOrder
    h_d = h_d + (theta^i)*coeff(i+1);
end
d_h_d_dtheta = jacobian(h_d,theta);

% TODO finish IO feedback linearization


