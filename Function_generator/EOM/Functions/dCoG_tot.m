function dCoG_tot = dCoG_tot(in1,in2)
%DCOG_TOT
%    DCOG_TOT = DCOG_TOT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    30-Nov-2018 22:29:06

alphaL = in1(6,:);
alphaR = in1(4,:);
betaL = in1(7,:);
betaR = in1(5,:);
l2 = in2(:,6);
l3 = in2(:,10);
lH = in2(:,3);
lL2 = in2(:,7);
m1 = in2(:,1);
m2 = in2(:,4);
m3 = in2(:,8);
phi = in1(3,:);
valphaL = in1(13,:);
valphaR = in1(11,:);
vbetaL = in1(14,:);
vbetaR = in1(12,:);
vphi = in1(10,:);
vx = in1(8,:);
vy = in1(9,:);
t2 = alphaL+phi;
t3 = cos(t2);
t4 = alphaR+phi;
t5 = cos(t4);
t6 = cos(phi);
t7 = alphaL+betaL+phi;
t8 = cos(t7);
t9 = alphaR+betaR+phi;
t10 = cos(t9);
t11 = m2.*2.0;
t12 = m3.*2.0;
t13 = m1+t11+t12;
t14 = 1.0./t13;
t15 = sin(t2);
t16 = sin(t4);
t17 = sin(phi);
t18 = sin(t7);
t19 = sin(t9);
dCoG_tot = [t14.*(m1.*vx+m2.*vx.*2.0+m3.*vx.*2.0+l2.*m2.*t3.*valphaL+l3.*m3.*t8.*valphaL+l2.*m2.*t5.*valphaR+l3.*m3.*t10.*valphaR+l3.*m3.*t8.*vbetaL+l3.*m3.*t10.*vbetaR+l2.*m2.*t3.*vphi+l2.*m2.*t5.*vphi+l3.*m3.*t8.*vphi+l3.*m3.*t10.*vphi+lL2.*m3.*t3.*valphaL+lL2.*m3.*t5.*valphaR+lH.*m2.*t6.*vphi.*2.0+lH.*m3.*t6.*vphi.*2.0+lL2.*m3.*t3.*vphi+lL2.*m3.*t5.*vphi);t14.*(m1.*vy+m2.*vy.*2.0+m3.*vy.*2.0+l2.*m2.*t15.*valphaL+l3.*m3.*t18.*valphaL+l2.*m2.*t16.*valphaR+l3.*m3.*t19.*valphaR+l3.*m3.*t18.*vbetaL+l3.*m3.*t19.*vbetaR+l2.*m2.*t15.*vphi+l2.*m2.*t16.*vphi+l3.*m3.*t18.*vphi+l3.*m3.*t19.*vphi+lL2.*m3.*t15.*valphaL+lL2.*m3.*t16.*valphaR+lH.*m2.*t17.*vphi.*2.0+lH.*m3.*t17.*vphi.*2.0+lL2.*m3.*t15.*vphi+lL2.*m3.*t16.*vphi)];
