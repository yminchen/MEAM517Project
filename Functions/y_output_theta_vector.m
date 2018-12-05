function y = y_output_theta_vector(in1,theta,in3)
%Y_OUTPUT_THETA_VECTOR
%    Y = Y_OUTPUT_THETA_VECTOR(IN1,THETA,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    04-Dec-2018 19:40:29

PI = in3(:,13);
qm2 = in1(2,:);
qm3 = in1(3,:);
qm4 = in1(4,:);
qm5 = in1(5,:);
t2 = theta.^2;
t3 = t2.^2;
t4 = t3.^2;
y = [PI.*2.0-qm3+t2.*2.346089030123536-t3.*1.957393129583595e2-t4.*2.992720727186096e4-theta.*2.734359794930546e-1+t2.*t3.*4.496012979958705e3-t2.*t4.*1.648471598403646e5+t3.*t4.*2.15640473846542e6+t2.*theta.*5.348392453506084e1-t3.*theta.*1.649424084026064e3-t4.*theta.*3.90044706913157e5+t2.*t3.*theta.*3.530884275686678e4+t2.*t4.*theta.*1.687756811140596e6-2.390318593611907e-1;-qm2-t2.*4.52192092614365e-1+t3.*2.372235291914007e1-t4.*2.090795682978566e5+theta.*1.916406413515567e-3+t2.*t3.*5.316743542578143e3+t2.*t4.*2.781979040297369e6-t3.*t4.*1.298972869491902e7-t2.*theta.*3.874978675513356+t3.*theta.*7.662282655424715e2+t4.*theta.*3.941621095945704e5-t2.*t3.*theta.*2.832433742485333e4-t2.*t4.*theta.*1.936794608684202e6+1.749739727292305e-1;qm4+t2.*4.000490555716192e1-t3.*2.01290026712641e3-t4.*1.113012190655174e6-theta.*7.751170673699094+t2.*t3.*6.307607809103893e4+t2.*t4.*1.011204675939848e7-t3.*t4.*3.669892488065831e7+t2.*theta.*2.963387703090161e2-t3.*theta.*7.072139897608828e3-t4.*theta.*8.378968167115839e5+t2.*t3.*theta.*1.052145748377265e5+t2.*t4.*theta.*2.70037581727408e6-7.429399966994999e-1;qm5-t2.*9.252751287078597e1+t3.*4.334562836845612e3+t4.*2.449716210730867e6+theta.*2.768293510212255-t2.*t3.*1.330903237775591e5-t2.*t4.*2.407666829542686e7+t3.*t4.*9.624331872119467e7-t2.*theta.*6.074750526969589-t3.*theta.*6.015452846172587e3-t4.*theta.*2.924686355503235e6+t2.*t3.*theta.*2.138879401516021e5+t2.*t4.*theta.*1.427542623942799e7+1.38988333442842];
