function CoGStanceShank = CoGStanceShank_MinCoord(in1,in2)
%COGSTANCESHANK_MINCOORD
%    COGSTANCESHANK = COGSTANCESHANK_MINCOORD(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    03-Dec-2018 11:57:17

l3 = in2(:,10);
lL3 = in2(:,11);
qm1 = in1(1,:);
t2 = l3-lL3;
CoGStanceShank = [t2.*sin(qm1);-t2.*cos(qm1)];
