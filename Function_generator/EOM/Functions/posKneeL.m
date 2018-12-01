function KneeL = posKneeL(in1,in2)
%POSKNEEL
%    KNEEL = POSKNEEL(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    30-Nov-2018 22:29:04

alphaL = in1(6,:);
lH = in2(:,3);
lL2 = in2(:,7);
phi = in1(3,:);
x = in1(1,:);
y = in1(2,:);
t2 = alphaL+phi;
KneeL = [x+lH.*sin(phi)+lL2.*sin(t2);y-lH.*cos(phi)-lL2.*cos(t2)];
