% Test whether the auto-generated function is correct 
addpath('Functions');
addpath('../..');   % for parameters

param = yumingParameters();
sysParam = param.sysParam;
sysParam_minCoord = param.sysParam_minCoord;

%% Check the position functions and the conversion from floating base to minimal coord

% sample configuration (minimal coord)
qm = [30;30;270;30;-30]*pi/180; 

% sample configuration (floating base) (assume right stance)
q = [  -0.1045;
        0.5573;
       -0.1295;
        0.4598;
       -0.0873;
       -0.0483;
       -0.2247];
q_float_partial = q([5,4,3,6,7]); 
qm = q_MinCoord(q_float_partial, pi);

CoGStanceShank = CoGStanceShank_MinCoord(qm,sysParam_minCoord);
StanceKnee = StanceKnee_MinCoord(qm,sysParam_minCoord);
CoGStanceThigh = CoGStanceThigh_MinCoord(qm,sysParam_minCoord);
Hip = Hip_MinCoord(qm,sysParam_minCoord);
CoGB = CoGB_MinCoord(qm,sysParam_minCoord);
CoGSwingThigh = CoGSwingThigh_MinCoord(qm,sysParam_minCoord);
SwingKnee = SwingKnee_MinCoord(qm,sysParam_minCoord);
CoGSwingShank = CoGSwingShank_MinCoord(qm,sysParam_minCoord);
SwingFoot = SwingFoot_MinCoord(qm,sysParam_minCoord);

figure; hold on;
scatter(0,0, 'bo')
scatter(StanceKnee(1), StanceKnee(2), 'bo')
scatter(Hip(1), Hip(2), 'bo')
scatter(CoGB(1), CoGB(2), 'bo')
scatter(SwingKnee(1), SwingKnee(2), 'bo')
scatter(SwingFoot(1), SwingFoot(2), 'bo')
scatter(CoGStanceShank(1), CoGStanceShank(2), 'rx')
scatter(CoGStanceThigh(1), CoGStanceThigh(2), 'rx')
scatter(CoGSwingThigh(1), CoGSwingThigh(2), 'rx')
scatter(CoGSwingShank(1), CoGSwingShank(2), 'rx')
xlim([-0.5, 0.5])
ylim([-0.2, 0.7])

%%
