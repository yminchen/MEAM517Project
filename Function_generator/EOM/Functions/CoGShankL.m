function CoGSL = CoGShankL(in1,in2)
%COGSHANKL
%    COGSL = COGSHANKL(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    02-Apr-2017 20:04:13

alphaL = in1(6,:);
betaL = in1(7,:);
l3 = in2(:,10);
lH = in2(:,3);
lL2 = in2(:,7);
phi = in1(3,:);
x = in1(1,:);
y = in1(2,:);
t2 = alphaL+phi;
t3 = alphaL+betaL+phi;
CoGSL = [x+lH.*sin(phi)+l3.*sin(t3)+lL2.*sin(t2);y-lH.*cos(phi)-l3.*cos(t3)-lL2.*cos(t2)];
