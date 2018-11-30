function [F,G] = userFuncXSquare(z)
    
    %% F
    % Dynamics Constraints
    
    % Cost
    F = z^2;
    
    %% G
    G = 2*z;
    
    
    %% Testing
    F = (z-1)^2;
    G = 2*(z-1);
    
    
end