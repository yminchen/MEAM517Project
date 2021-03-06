function CoGB = CoGB_MinCoord(in1,in2)
%COGB_MINCOORD
%    COGB = COGB_MINCOORD(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    03-Dec-2018 11:57:18

PI = in2(:,13);
lH = in2(:,3);
lL2 = in2(:,7);
lL3 = in2(:,11);
qm1 = in1(1,:);
qm2 = in1(2,:);
qm3 = in1(3,:);
t2 = qm1+qm2;
t3 = PI.*-2.0+qm1+qm2+qm3;
CoGB = [-lL3.*sin(qm1)-lH.*sin(t3)-lL2.*sin(t2);lL3.*cos(qm1)+lH.*cos(t3)+lL2.*cos(t2)];
