% double integrator example (test the usage of SNOPT)
clear all

%%
N = 1;
nF = 1;

%%
ObjRow = 1;

ObjAdd = 0;

z		= zeros(N,1);
zstate	= zeros(N,1);
zmul	= zeros(N,1);
zlow	= -inf*ones(N,1);
zupp	=  inf*ones(N,1);

Fstate	= zeros(nF,1);
Fmul	= zeros(nF,1);
Flow	= -inf*ones(nF,1);
Fupp	=  inf*ones(nF,1);

%%
z(1) = 1;

%%
A = zeros(0,N);
iAfun = NaN(0,N);
jAvar = NaN(0,N);

%%
userfun = @(z) userFuncXSquare(z);
iGfun = 1;
jGvar = 1;

%%
snscreen on; % print to command screen

options.name = 'XSquare';

% [z,F,inform,xmul,Fmul,xstate,Fstate,output] = ...
%     snopt( z, zlow, zupp, zmul, zstate, ...
%     Flow, Fupp, Fmul, Fstate, ...
%     userfun, ObjAdd, ObjRow, options );

[z,F,inform,xmul,Fmul,xstate,Fstate,output] = ...
    snopt( z, zlow, zupp, zmul, zstate, ...
    Flow, Fupp, Fmul, Fstate, ...
    userfun, ObjAdd, ObjRow, ...
    A, iAfun, jAvar, iGfun, jGvar, [options] );

disp(['solution = ', num2str(z)]);

