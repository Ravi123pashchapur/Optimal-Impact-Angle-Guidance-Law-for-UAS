function AM = Guidance(R1, SIG1, GAM1, SIGR1)
global GAMD VM1
    
%.. Guidance Law
    % Parameters
    % Time to Go
    Tgo = R1/VM1;
    R_dot = -VM1*cos(GAMD-SIG1);
    
    %Lateral position 
    y = R1*(GAMD-SIG1);
    
    % Lateral Velocity
    v = VM1*(GAM1-GAMD);
    
    % Guidance Command
    % Lateral Acceleration
    AM = ((-6*y)/Tgo^2)-((4*v)/Tgo);
end