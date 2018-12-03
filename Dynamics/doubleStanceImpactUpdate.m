function [dqplus,nextPhase] = doubleStanceImpactUpdate(x,phase)
    param = yumingParameters();
    sysParam = param.sysParam;
    
    n = size(x,1);
    M = MassMatrix(x(1:n/2),sysParam);
    JR = JcontPointR(x(1:n/2),sysParam);
    JL = JcontPointL(x(1:n/2),sysParam);
    
    % First assume that it's double stance, then check if the impulse on 
    % the original stance leg is positive. 
    % If it's negative (stiction), then it means that the
    % original stance leg leaves the ground.
    
    J = [JR;JL];
    LAMBDA = -(J*(M\J'))\(J*x(n/2+1:n));
    if (phase == 2 || phase == 3)
        if(LAMBDA(2) < -0.001) % can't put 0, cause numerical error!
            disp('Left Foot Touch Down. Swap stance leg.');
            nextPhase = 5;
            J = JL;
            dqplus = (eye(n/2) - (M\J')*pinv(J*(M\J'))*J)*x(n/2+1:n);
            % dqplus_LAMBDA = [M -J';J zeros(2)]\[M*x(n/2+1:n);zeros(2,1)];
            
%             Testing
%             velFootR = JR*dqplus;
%             disp('Testing velFootR_y (should be positve): ');
%             disp(velFootR(2));
        else
            disp('Left Foot Touch Down. Enter double stance');
            nextPhase = 8;
            dqplus = (eye(n/2) - (M\J')*pinv(J*(M\J'))*J)*x(n/2+1:n);
        end 
    elseif (phase == 5 || phase == 6)
        if(LAMBDA(4) < -0.001)
            disp('Right Foot Touch Down. Swap stance leg.');
            nextPhase = 2;
            J = JR;
            dqplus = (eye(n/2) - (M\J')*pinv(J*(M\J'))*J)*x(n/2+1:n);
            
%             Testing
%             velFootL = JL*dqplus;
%             disp('Testing velFootL_y (should be positve): ');
%             disp(velFootL(2));
        else
            disp('Right Foot Touch Down. Enter double stance');
            nextPhase = 7;
            dqplus = (eye(n/2) - (M\J')*pinv(J*(M\J'))*J)*x(n/2+1:n);
        end
    elseif (phase == 1 || phase == 4)   
        nextPhase = 7;
        dqplus = (eye(n/2) - (M\J')*pinv(J*(M\J'))*J)*x(n/2+1:n);
    else
        disp('Error (In doubleStanceImpactUpdate)');
    end
        
    
end