function thetaL_BF = ThetaL_BF(in1,in2)
%THETAL_BF
%    THETAL_BF = THETAL_BF(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    30-Nov-2018 22:29:14

alphaL = in1(6,:);
betaL = in1(7,:);
lH = in2(:,3);
lL2 = in2(:,7);
lL3 = in2(:,11);
phi = in1(3,:);
t2 = alphaL+phi;
t3 = alphaL+betaL+phi;
thetaL_BF = angle(lH.*cos(phi)+lL2.*cos(t2)+lL3.*cos(t3)+lH.*sin(phi).*1i+lL2.*sin(t2).*1i+lL3.*sin(t3).*1i);
