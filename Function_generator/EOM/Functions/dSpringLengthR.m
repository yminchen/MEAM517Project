function dLengthR = dSpringLengthR(in1,in2)
%DSPRINGLENGTHR
%    DLENGTHR = DSPRINGLENGTHR(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    02-Apr-2017 20:04:17

alphaR = in1(4,:);
betaR = in1(5,:);
lH = in2(:,3);
lL2 = in2(:,7);
lL3 = in2(:,11);
valphaR = in1(11,:);
vbetaR = in1(12,:);
t2 = alphaR+betaR;
t3 = sin(t2);
dLengthR = -(lH.*lL3.*t3.*valphaR+lH.*lL3.*t3.*vbetaR+lH.*lL2.*valphaR.*sin(alphaR)+lL2.*lL3.*vbetaR.*sin(betaR)).*1.0./sqrt(lH.^2+lL2.^2+lL3.^2+lH.*lL3.*cos(t2).*2.0+lH.*lL2.*cos(alphaR).*2.0+lL2.*lL3.*cos(betaR).*2.0);
