  
%.. Declare Global Variables

    % Global Constants
    global          r2d             d2r         DT
    
    % Guidance and Control Parameters
    global          GAMD            VM1

%.. Define Variables for Global Constant

    r2d                 =       180 / pi ;                                % Radian to Degree                          (ND)
    d2r                 =       pi / 180 ;                                % Degree to Radian    
    DT                  =       0.001 ;                                   % Integration time step (sec)

%.. Define Missile Parameters
    
    VMX10               =       10 * cos(20 * d2r);                       % Missile X Velocity                        (m/s)
    VMY10               =       10 * sin(20 * d2r);                       % Missile Y Velocity                        (m/s)
    VM1                 =       sqrt( VMX10^2 + VMY10^2 ) ;               % Missile Velocity                          (m/s)
    MX10                =       0 ;                                       % Missile X Position                        (m)
    MY10                =       0 ;                                       % Missile Y Position                        (m)
    GAM_M10             =       atan2( VMY10, VMX10 ) ;                   % Missile Flight Path Angle                 (rad)

%.. Define Target Parameters    
    
    TX0                 =       20 ;                                      % Target X Position                         (m)
    TY0                 =       0 ;                                       % Target Y Position                         (m)
    
%.. Parameters for Guidance

    GAMD                =       0 * d2r ;                                 % Desired Terminal Flight Path Angle        (rad)