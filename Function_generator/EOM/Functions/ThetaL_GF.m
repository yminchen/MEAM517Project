function thetaL_GF = ThetaL_GF(in1,in2)
%THETAL_GF
%    THETAL_GF = THETAL_GF(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    02-Apr-2017 20:04:18

alphaL = in1(6,:);
alphaR = in1(4,:);
betaL = in1(7,:);
betaR = in1(5,:);
l2 = in2(:,6);
l3 = in2(:,10);
lH = in2(:,3);
lL2 = in2(:,7);
lL3 = in2(:,11);
m1 = in2(:,1);
m2 = in2(:,4);
m3 = in2(:,8);
phi = in1(3,:);
t2 = alphaL+betaL+phi;
t3 = cos(t2);
t4 = alphaR+betaR+phi;
t5 = sin(t2);
t6 = alphaL+phi;
t7 = cos(t6);
t8 = alphaR+phi;
t9 = cos(t8);
t10 = sin(t6);
t11 = sin(t8);
thetaL_GF = angle((lH.*m1.*cos(phi)-l3.*m3.*cos(t4)+lH.*m1.*sin(phi).*1i-l3.*m3.*sin(t4).*1i-l3.*m3.*t3-l2.*m2.*t7-l3.*m3.*t5.*1i-l2.*m2.*t9-l2.*m2.*t10.*1i-l2.*m2.*t11.*1i+lL3.*m1.*t3+lL3.*m2.*t3.*2.0+lL3.*m1.*t5.*1i+lL3.*m3.*t3.*2.0+lL2.*m1.*t7+lL3.*m2.*t5.*2.0i+lL2.*m2.*t7.*2.0+lL3.*m3.*t5.*2.0i+lL2.*m3.*t7+lL2.*m1.*t10.*1i+lL2.*m2.*t10.*2.0i-lL2.*m3.*t9+lL2.*m3.*t10.*1i-lL2.*m3.*t11.*1i)./(m1+m2.*2.0+m3.*2.0));
