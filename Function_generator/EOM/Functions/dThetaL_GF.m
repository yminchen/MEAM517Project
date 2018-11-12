function dthetaL_GF = dThetaL_GF(in1,in2)
%DTHETAL_GF
%    DTHETAL_GF = DTHETAL_GF(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    02-Apr-2017 20:04:20

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
valphaL = in1(13,:);
valphaR = in1(11,:);
vbetaL = in1(14,:);
vbetaR = in1(12,:);
vphi = in1(10,:);
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
t13 = m2.*2.0;
t14 = m3.*2.0;
t15 = m1+t13+t14;
t16 = 1.0./t15;
t17 = cos(phi);
t18 = lH.*m1.*t17;
t19 = sin(phi);
t20 = lH.*m1.*t19.*1i;
t21 = l3.*m3.*t3;
t22 = cos(t4);
t23 = l3.*m3.*t22;
t24 = lL3.*m1.*t3;
t25 = lL3.*m2.*t3.*2.0;
t26 = lL3.*m3.*t3.*2.0;
t27 = l3.*m3.*t5.*-1i;
t28 = sin(t4);
t88 = l3.*m3.*t28.*1i;
t85 = -t88;
t82 = -t85;
t29 = -t82;
t30 = lL3.*m1.*t5.*1i;
t31 = lL3.*m2.*t5.*2.0i;
t32 = lL3.*m3.*t5.*2.0i;
t33 = l2.*m2.*t7;
t34 = l2.*m2.*t9;
t35 = lL2.*m1.*t7;
t36 = lL2.*m2.*t7.*2.0;
t37 = lL2.*m3.*t7;
t38 = lL2.*m3.*t9;
t39 = l2.*m2.*t10.*-1i;
t89 = l2.*m2.*t11.*1i;
t86 = -t89;
t83 = -t86;
t40 = -t83;
t41 = lL2.*m1.*t10.*1i;
t42 = lL2.*m2.*t10.*2.0i;
t43 = lL2.*m3.*t10.*1i;
t90 = lL2.*m3.*t11.*1i;
t87 = -t90;
t84 = -t87;
t44 = -t84;
t45 = t18+t20-t21-t23+t24+t25+t26+t27+t29+t30+t31+t32-t33-t34+t35+t36+t37-t38+t39+t40+t41+t42+t43+t44;
t46 = t16.*t45;
t12 = imag(t46);
t47 = real(t46);
t48 = t22.*1i;
t49 = t28-t48;
t50 = l3.*m3.*t16.*t49;
t51 = lH.*m1.*t19;
t52 = lL3.*m1.*t5;
t53 = lL3.*m2.*t5.*2.0;
t54 = lL3.*m3.*t5.*2.0;
t55 = lL2.*m1.*t10;
t56 = lL2.*m2.*t10.*2.0;
t57 = lL2.*m3.*t10;
t58 = l3.*m3.*t3.*1i;
t59 = l2.*m2.*t7.*1i;
t60 = l2.*m2.*t9.*1i;
t61 = l3.*m3.*t22.*1i;
t62 = lH.*m1.*t17.*-1i;
t63 = lL3.*m1.*t3.*-1i;
t64 = lL3.*m2.*t3.*-2.0i;
t65 = lL3.*m3.*t3.*-2.0i;
t66 = lL2.*m1.*t7.*-1i;
t67 = lL2.*m2.*t7.*-2.0i;
t68 = lL2.*m3.*t7.*-1i;
t69 = lL2.*m3.*t9.*1i;
t71 = l3.*m3.*t5;
t72 = l2.*m2.*t10;
t70 = t51+t52+t53+t54+t55+t56+t57+t58+t59+t60+t61+t62+t63+t64+t65+t66+t67+t68+t69-t71-t72-l2.*m2.*t11-l3.*m3.*t28-lL2.*m3.*t11;
t73 = t52+t53+t54+t55+t56+t57+t58+t59+t63+t64+t65+t66+t67+t68-t71-t72;
t74 = t16.*t73;
t75 = lL3.*m1;
t76 = lL3.*m2.*2.0;
t77 = lL3.*m3.*2.0;
t78 = t75+t76+t77-l3.*m3;
t79 = t3.*1i;
t80 = t5-t79;
t81 = t16.*t78.*t80;
dthetaL_GF = -(t47.*valphaL.*imag(t74)-t47.*vbetaR.*imag(t50)+t47.*vbetaL.*imag(t81)-t12.*valphaL.*real(t74)+t12.*vbetaR.*real(t50)-t12.*vbetaL.*real(t81)+valphaR.*real(t16.*(t23+t34+t38+t88+t89+t90)).*real(t16.*(t18+t20-t21-t23+t24+t25+t26+t27+t30+t31+t32-t33-t34+t35+t36+t37-t38+t39+t41+t42+t43+t85+t86+t87))+t12.*valphaR.*imag(t16.*(t23+t34+t38+t82+t83+t84))-t47.*vphi.*imag(-t16.*t70)+t12.*vphi.*real(-t16.*t70))./(t12.^2+t47.^2);
