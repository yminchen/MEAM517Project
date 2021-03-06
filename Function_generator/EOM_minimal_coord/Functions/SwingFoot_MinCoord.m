function SwingFoot = SwingFoot_MinCoord(in1,in2)
%SWINGFOOT_MINCOORD
%    SWINGFOOT = SWINGFOOT_MINCOORD(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    03-Dec-2018 11:57:19

PI = in2(:,13);
lL2 = in2(:,7);
lL3 = in2(:,11);
qm1 = in1(1,:);
qm2 = in1(2,:);
qm3 = in1(3,:);
qm4 = in1(4,:);
qm5 = in1(5,:);
t4 = PI.*2.0;
t2 = qm1+qm2+qm3+qm4-t4;
t3 = qm1+qm2;
SwingFoot = [lL3.*sin(PI.*-2.0+qm1+qm2+qm3+qm4+qm5)-lL3.*sin(qm1)+lL2.*sin(t2)-lL2.*sin(t3);-lL3.*cos(qm1+qm2+qm3+qm4+qm5-t4)+lL3.*cos(qm1)-lL2.*cos(t2)+lL2.*cos(t3)];
