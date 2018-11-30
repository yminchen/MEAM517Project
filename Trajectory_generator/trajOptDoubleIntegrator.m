% double integrator example (test the usage of SNOPT)
clear all

%%
n_sample = 100;
n_x = 2;
n_u = 1;
T = 1; % (seconds)

n_X = n_x * n_sample;
n_U = (n_sample-1)*n_u;

N = n_X + n_U;

%%
nCon   = n_x*(n_sample-1);
nF     = nCon + 1;

Obj    = nF;
ObjRow = Obj;

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
z(2) = 1;

%%
zlow(n_X+[1:n_U],1) = -1;
zupp(n_X+[1:n_U],1) =  1;

Flow(1:nCon) = 0;    Fupp(1:nCon) = 0;  
Flow(ObjRow) = -inf; Fupp(ObjRow) = inf;

%%
A = zeros(0,N);
iAfun = NaN(0,N);
jAvar = NaN(0,N);

%%


% TODO: Need to implement derivative



userfun = @(z) userFuncDoubleIntegrator(z,n_sample,n_x,n_u,T);
% for i = 1:(5*nCon+2)
%     iGfun(i)
% end
% iGfun = NaN;
% jGvar = NaN;

% iGfun = []; jGvar = [];
% for i = 1:nF
%     for j = 1:N
%         iGfun = [iGfun;i];
%         jGvar = [jGvar;j];
%     end
% end


%%
snscreen on; % print to command screen
options.name = 'DoubleIntegrator';

[z,F,inform,xmul,Fmul,xstate,Fstate,output] = ...
    snopt( z, zlow, zupp, zmul, zstate, ...
    Flow, Fupp, Fmul, Fstate, ...
    userfun, ObjAdd, ObjRow, options );

% [z,F,inform,xmul,Fmul,xstate,Fstate,output] = ...
%     snopt( z, zlow, zupp, zmul, zstate, ...
%     Flow, Fupp, Fmul, Fstate, ...
%     userfun, ObjAdd, ObjRow, ...
%     A, iAfun, jAvar, iGfun, jGvar, [options] );


