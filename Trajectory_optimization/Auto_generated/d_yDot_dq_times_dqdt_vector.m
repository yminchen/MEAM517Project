function d_yDot_dq_times_dqdt = d_yDot_dq_times_dqdt_vector(in1,in2)
%D_YDOT_DQ_TIMES_DQDT_VECTOR
%    D_YDOT_DQ_TIMES_DQDT = D_YDOT_DQ_TIMES_DQDT_VECTOR(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    02-Dec-2018 23:03:09

dqm1 = in1(6,:);
dqm2 = in1(7,:);
qm1 = in1(1,:);
qm2 = in1(2,:);
t3 = qm2.*(1.0./2.0);
t2 = qm1+t3;
t4 = t2.^2;
t5 = t4.^2;
t6 = t5.^2;
t7 = qm1.*1.693701456264893e2;
t8 = qm2.*8.468507281324464e1;
t9 = t4.*3.719538696072758e2;
t10 = t2.*t5.*1.234779267080217e6;
t11 = t6.*5.180337679991068e7;
t12 = t2.*t6.*1.768103041512518e8;
t13 = t5.*(-1.456842577315813e4)+t7+t8+t9+t10+t11+t12-t2.*t4.*2.345779483922437e4-t4.*t5.*1.363328298453494e6-t4.*t6.*4.636083918056729e8-t2.*t4.*t5.*2.539754160336886e7+2.290991246757058;
t14 = qm1.*3.59020474280434e1;
t15 = qm2.*1.79510237140217e1;
t16 = t4.*2.062492845781417e3;
t17 = t2.*t5.*1.058403896977055e6;
t18 = t4.*t5.*7.264505424988172e5;
t19 = t6.*3.57691752743535e7;
t20 = t2.*t6.*1.693638009680625e8;
t21 = t5.*(-1.160423914047124e5)+t14+t15+t16+t17+t18+t19+t20-t2.*t4.*1.549635206523476e4-t4.*t6.*4.446149260293051e8-t2.*t4.*t5.*2.360911271000695e7-1.77927349459624;
t22 = qm1.*1.050876906752897e3;
t23 = qm2.*5.254384533764484e2;
t24 = t4.*2.304923270854616e3;
t25 = t2.*t5.*5.185399015019607e6;
t26 = t4.*t5.*1.26746599002504e5;
t27 = t6.*5.853532301599879e7;
t28 = t2.*t6.*6.123919398030727e8;
t29 = t5.*(-1.082677428925182e5)+t22+t23+t24+t25+t26+t27+t28-t2.*t4.*1.20244937578412e5-t4.*t6.*6.781557208774573e8-t2.*t4.*t5.*9.454173129878535e7-7.745790474713721;
t30 = qm1.*1.096702279469362e3;
t31 = qm2.*5.483511397346811e2;
t32 = t4.*1.935205612854446e4;
t33 = t2.*t5.*5.572612409234471e6;
t34 = t4.*t5.*4.190047953061355e7;
t35 = t2.*t6.*5.490461849128678e8;
t36 = t4.*t6.*2.808214439343077e9;
t37 = t5.*(-1.378424761450081e6)-t6.*5.685692777921415e8+t30+t31+t32+t33+t34+t35+t36-t2.*t4.*1.373813318661763e5-t2.*t4.*t5.*9.271707491293113e7-6.653005684691809e1;
d_yDot_dq_times_dqdt = [-dqm1.*(dqm1.*(qm1.*3.387402912529786e2+qm2.*1.693701456264893e2+t4.*7.439077392145516e2-t5.*2.913685154631626e4+t6.*1.036067535998214e8-t2.*t4.*4.691558967844873e4+t2.*t5.*2.469558534160433e6+t2.*t6.*3.536206083025036e8-t4.*t5.*2.726656596906987e6-t4.*t6.*9.272167836113458e8-t2.*t4.*t5.*5.079508320673771e7+4.581982493514116)+dqm2.*t13)-dqm2.*(dqm1.*t13+dqm2.*(qm1.*8.468507281324464e1+qm2.*4.234253640662232e1+t4.*1.859769348036379e2-t5.*7.284212886579066e3+t6.*2.590168839995534e7-t2.*t4.*1.172889741961218e4+t2.*t5.*6.173896335401083e5+t2.*t6.*8.84051520756259e7-t4.*t5.*6.816641492267468e5-t4.*t6.*2.318041959028365e8-t2.*t4.*t5.*1.269877080168443e7+1.145495623378529));dqm1.*(dqm2.*t21+dqm1.*(qm1.*7.18040948560868e1+qm2.*3.59020474280434e1+t4.*4.124985691562835e3-t5.*2.320847828094248e5+t6.*7.1538350548707e7-t2.*t4.*3.099270413046951e4+t2.*t5.*2.11680779395411e6+t2.*t6.*3.38727601936125e8+t4.*t5.*1.452901084997634e6-t4.*t6.*8.892298520586102e8-t2.*t4.*t5.*4.72182254200139e7-3.55854698919248))+dqm2.*(dqm1.*t21+dqm2.*(qm1.*1.79510237140217e1+qm2.*8.97551185701085+t4.*1.031246422890709e3-t5.*5.802119570235621e4+t6.*1.788458763717675e7-t2.*t4.*7.748176032617379e3+t2.*t5.*5.292019484885275e5+t2.*t6.*8.468190048403125e7+t4.*t5.*3.632252712494086e5-t4.*t6.*2.223074630146525e8-t2.*t4.*t5.*1.180455635500348e7-8.896367472981199e-1));-dqm2.*(dqm2.*(qm1.*5.254384533764484e2+qm2.*2.627192266882242e2+t4.*1.152461635427308e3-t5.*5.413387144625909e4+t6.*2.92676615079994e7-t2.*t4.*6.0122468789206e4+t2.*t5.*2.592699507509803e6+t2.*t6.*3.061959699015363e8+t4.*t5.*6.337329950125201e4-t4.*t6.*3.390778604387287e8-t2.*t4.*t5.*4.727086564939267e7-3.872895237356861)+dqm1.*t29)-dqm1.*(dqm1.*(qm1.*2.101753813505793e3+qm2.*1.050876906752897e3+t4.*4.609846541709231e3-t5.*2.165354857850363e5+t6.*1.170706460319976e8-t2.*t4.*2.40489875156824e5+t2.*t5.*1.037079803003921e7+t2.*t6.*1.224783879606145e9+t4.*t5.*2.534931980050081e5-t4.*t6.*1.356311441754915e9-t2.*t4.*t5.*1.890834625975707e8-1.549158094942744e1)+dqm2.*t29);dqm2.*(dqm1.*t37+dqm2.*(qm1.*5.483511397346811e2+qm2.*2.741755698673406e2+t4.*9.676028064272232e3-t5.*6.892123807250407e5-t6.*2.842846388960708e8-t2.*t4.*6.869066593308813e4+t2.*t5.*2.786306204617235e6+t2.*t6.*2.745230924564339e8+t4.*t5.*2.095023976530678e7+t4.*t6.*1.404107219671538e9-t2.*t4.*t5.*4.635853745646557e7-3.326502842345904e1))+dqm1.*(dqm2.*t37+dqm1.*(qm1.*2.193404558938724e3+qm2.*1.096702279469362e3+t4.*3.870411225708893e4-t5.*2.756849522900163e6-t6.*1.137138555584283e9-t2.*t4.*2.747626637323525e5+t2.*t5.*1.114522481846894e7+t2.*t6.*1.098092369825736e9+t4.*t5.*8.380095906122711e7+t4.*t6.*5.616428878686153e9-t2.*t4.*t5.*1.854341498258623e8-1.330601136938362e2))];
