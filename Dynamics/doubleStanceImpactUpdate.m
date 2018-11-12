function [dqplus,nextPhase] = doubleStanceImpactUpdate(x,phase)
    param = yumingParameters();
    sysParam = param.sysParam;
    
    n = size(x,1);
    M = MassMatrix(x(1:n/2),sysParam);
    
    JR = JcontPointR(x(1:n/2),sysParam);
    JL = JcontPointL(x(1:n/2),sysParam);
    
    J = [JR;JL];
    LAMBDA = -(J*(M\J'))\(J*x(n/2+1:n));
    if (phase == 2 || phase == 3)
        if(LAMBDA(4)>0)
            nextPhase = 5;
            

            J = JL;
            dqplus = (eye(n/2) - (M\J')*pinv(J*(M\J'))*J)*x(n/2+1:n);
            velFootR = JR*dqplus;
            disp('Testing velFootR_y (should be positve?): ');
            disp(velFootR(2));
            
            
        else
            nextPhase = 8;
        end 
    elseif (phase == 5 || phase == 6)
        if(LAMBDA(2)>0)
            nextPhase = 2;
            
            
            
            J = JR;
            dqplus = (eye(n/2) - (M\J')*pinv(J*(M\J'))*J)*x(n/2+1:n);
            velFootL = JL*dqplus;
            disp('Testing velFootL_y (should be positve?): ');
            disp(velFootL(2));
            
            
            
            
        else
            nextPhase = 7;
        end 
    else
        disp('In doubleStanceImpactUpdate, error');
    end
        
    
    dqplus = (eye(n/2) - (M\J')*pinv(J*(M\J'))*J)*x(n/2+1:n);
end