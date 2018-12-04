function d_yDot_ddq = d_yDot_ddq_theta_vector(in1,theta,in3)
%D_YDOT_DDQ_THETA_VECTOR
%    D_YDOT_DDQ = D_YDOT_DDQ_THETA_VECTOR(IN1,THETA,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    04-Dec-2018 17:19:10

t2 = theta.^2;
t3 = t2.^2;
t4 = t3.^2;
d_yDot_ddq = reshape([t2.*(-1.693701456264893e2)+t3.*1.172889741961218e4+t4.*6.349385400842214e6+theta.*4.581982493514116-t2.*t3.*4.115930890267389e5-t2.*t4.*3.536206083025036e7+t2.*theta.*2.479692464048505e2-t3.*theta.*5.827370309263253e3+t4.*theta.*1.151186151109126e7-t2.*t3.*theta.*3.895223709867125e5-t2.*t4.*theta.*8.429243487375871e7+1.516776637355894e-1,t2.*3.59020474280434e1-t3.*7.748176032617379e3-t4.*5.902278177501738e6+theta.*3.55854698919248+t2.*t3.*3.528012989923517e5+t2.*t4.*3.38727601936125e7-t2.*theta.*1.374995230520945e3+t3.*theta.*4.641695656188496e4-t4.*theta.*7.948705616522999e6-t2.*t3.*theta.*2.075572978568049e5+t2.*t4.*theta.*8.083907745987365e7-1.851290877078436e-2,t2.*(-1.050876906752897e3)+t3.*6.0122468789206e4+t4.*2.363543282469634e7-theta.*1.549158094942744e1-t2.*t3.*1.728466338339869e6-t2.*t4.*1.224783879606145e8+t2.*theta.*1.536615513903077e3-t3.*theta.*4.330709715700727e4+t4.*theta.*1.300784955911084e7+t2.*t3.*theta.*3.621331400071544e4-t2.*t4.*theta.*1.233010401595377e8+7.049863990769837,t2.*1.096702279469362e3-t3.*6.869066593308813e4-t4.*2.317926872823278e7+theta.*1.330601136938362e2+t2.*t3.*1.857537469744824e6+t2.*t4.*1.098092369825736e8-t2.*theta.*1.290137075236298e4+t3.*theta.*5.513699045800326e5+t4.*theta.*1.263487283982537e8-t2.*t3.*theta.*1.19715655801753e7-t2.*t4.*theta.*5.10584443516923e8-4.17112462597125,t2.*(-8.468507281324464e1)+t3.*5.864448709806092e3+t4.*3.174692700421107e6+theta.*2.290991246757058-t2.*t3.*2.057965445133694e5-t2.*t4.*1.768103041512518e7+t2.*theta.*1.239846232024253e2-t3.*theta.*2.913685154631626e3+t4.*theta.*5.755930755545631e6-t2.*t3.*theta.*1.947611854933562e5-t2.*t4.*theta.*4.214621743687936e7+7.583883186779468e-2,t2.*1.79510237140217e1-t3.*3.874088016308689e3-t4.*2.951139088750869e6+theta.*1.77927349459624+t2.*t3.*1.764006494961758e5+t2.*t4.*1.693638009680625e7-t2.*theta.*6.874976152604725e2+t3.*theta.*2.320847828094248e4-t4.*theta.*3.9743528082615e6-t2.*t3.*theta.*1.037786489284025e5+t2.*t4.*theta.*4.041953872993683e7-1.009256454385392,t2.*(-5.254384533764484e2)+t3.*3.0061234394603e4+t4.*1.181771641234817e7-theta.*7.745790474713721-t2.*t3.*8.642331691699345e5-t2.*t4.*6.123919398030727e7+t2.*theta.*7.683077569515385e2-t3.*theta.*2.165354857850363e4+t4.*theta.*6.503924779555422e6+t2.*t3.*theta.*1.810665700035772e4-t2.*t4.*theta.*6.165052007976885e7+3.524931995384919,t2.*5.483511397346811e2-t3.*3.434533296654407e4-t4.*1.158963436411639e7+theta.*6.653005684691809e1+t2.*t3.*9.287687348724118e5+t2.*t4.*5.490461849128678e7-t2.*theta.*6.450685376181488e3+t3.*theta.*2.756849522900163e5+t4.*theta.*6.317436419912684e7-t2.*t3.*theta.*5.985782790087651e6-t2.*t4.*theta.*2.552922217584615e8-2.085562312985625,-1.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0],[4,5]);