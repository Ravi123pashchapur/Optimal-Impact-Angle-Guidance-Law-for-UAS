%/////////////////////////////////////////////////////////////////////////%
%                                                                         %
%   - Name : Main.m                                                       %
%                                                                         %
%                   - Created by Lee, H. I.    28. 01. 2022.              %
%                                                                         %
%/////////////////////////////////////////////////////////////////////////%

%.. Matlab Initialise 
    clear all; clc; warning off; 

%% Simulation at 'GAMD = 0 degree'.
%.. Simulation Initialise
    Sim_Parameters_0_deg;
    missile_states = [MX10 MY10 VMX10 VMY10 0 0 GAM_M10];
    target_states  = [TX0 TY0];
    a1 =0;
    time = 0;
for t = 0:DT:10
    %.. Seeker
        [R1,SIG1,GAM1,SIGR1] = Seeker(target_states,missile_states(end,:));
    %.. Guidance Law
        AM = Guidance(R1, SIG1, GAM1, SIGR1);
        a1 = [a1;AM];
    %.. Missile Dynamics
        missile_states_update = Dynamics(missile_states(end,:),AM);
        missile_states = [missile_states;missile_states_update];
        time = [time;t];
    %.. Simulation End
        if R1 <=0.01
            break;
        end
end
%.. Plot
% design your own codes to plot the results

X1 = missile_states(:,1);
Y1 = missile_states(:,2);
time_1 = time;
Gamma_1 = missile_states(:,7) * r2d;


%% Simulation at 'GAMD = -45 degree'.
%.. Simulation Initialise
    Sim_Parameters_minus_45_deg;
    missile_states = [MX10 MY10 VMX10 VMY10 0 0 GAM_M10];
    target_states  = [TX0 TY0];
    a2 = 0;
    time = 0;
for t = 0:DT:10
    %.. Seeker
        [R1,SIG1,GAM1,SIGR1] = Seeker(target_states,missile_states(end,:));
    %.. Guidance Law
        AM = Guidance(R1, SIG1, GAM1, SIGR1);
        a2 = [a2;AM];
    %.. Missile Dynamics
        missile_states_update = Dynamics(missile_states(end,:),AM);
        missile_states = [missile_states;missile_states_update];
        time = [time;t];
    %.. Simulation End
        if R1 <=0.005
            break;
        end
end
%.. Plot
% design your own codes to plot the results

X2 = missile_states(:,1);
Y2 = missile_states(:,2);
time_2 = time;
Gamma_2 = missile_states(:,7) * r2d;

%% Simulation at 'GAMD = -90 degree'.
%.. Simulation Initialise
    Sim_Parameters_minus_90_deg;
    missile_states = [MX10 MY10 VMX10 VMY10 0 0 GAM_M10];
    target_states  = [TX0 TY0];
    a3 = 0;
    time = 0;
for t = 0:DT:10
    %.. Seeker
        [R1,SIG1,GAM1,SIGR1] = Seeker(target_states,missile_states(end,:));
    %.. Guidance Law
        AM = Guidance(R1, SIG1, GAM1, SIGR1);
        a3 = [a3;AM];
    %.. Missile Dynamics
        missile_states_update = Dynamics(missile_states(end,:),AM);
        missile_states = [missile_states;missile_states_update];
        time = [time;t];
    %.. Simulation End
        if R1 <=0.002
            break;
        end
end
%.. Plot
% design your own codes to plot the results

X3 = missile_states(:,1);
Y3 = missile_states(:,2);
time_3 = [time;2.4010];
Gamma_3 = [missile_states(:,7) * r2d;-89.87];




%% Simulator
for k = 1:length(X3)
    if k<=length(X1)
        plot(X1(k),Y1(k),'rx',LineWidth = 2,MarkerSize = 7)
        xlabel('X [m]');
        ylabel('Y [m]');
        title('Intercept Trajectories Simulator')
        hold on
        plot(X2(k),Y2(k),'rx',LineWidth = 2,MarkerSize = 7)
        plot(X3(k),Y3(k),'rx',LineWidth = 2,MarkerSize = 7)
        
    elseif k<=length(X2)
        
        plot(X2(k),Y2(k),'rx',LineWidth = 2,MarkerSize = 7)
        xlabel('X [m]');
        ylabel('Y [m]');
        title('Intercept Trajectories Simulator')
        hold on
        plot(X3(k),Y3(k),'rx',LineWidth = 2,MarkerSize = 7)
        
    else
        plot(X3(k),Y3(k),'rx',LineWidth = 2,MarkerSize = 7)
        xlabel('X [m]');
        ylabel('Y [m]');
        title('Intercept Trajectories Simulator')
       
    end
    hold on
    if k<=length(X1)
        text(X1(k),Y1(k)+0.2,'Missile(\gamma_f =0^o)')
        text(X2(k),Y2(k)+0.2,'Missile(\gamma_f =-45^o)')
        text(X3(k),Y3(k)+0.2,'Missile(\gamma_f =-90^o)')
    elseif k<=length(X2)
        text(X2(k),Y2(k)+0.2,'Missile(\gamma_f =-45^o)')
        text(X3(k),Y3(k)+0.2,'Missile(\gamma_f =-90^o)')
    else
        text(X3(k),Y3(k)+0.2,'Missile(\gamma_f =-90^o)')
    end
    if k<=length(X1)
        plot(X1(1:k),Y1(1:k),'m-.',LineWidth = 2)
        hold on
        plot(X2(1:k),Y2(1:k),'g-.',LineWidth = 2)
        plot(X3(1:k),Y3(1:k),'b-.',LineWidth = 2)
        
    elseif k<=length(X2)
        plot(X2(1:k),Y2(1:k),'g-.',LineWidth = 2)
        hold on
        plot(X3(1:k),Y3(1:k),'b-.',LineWidth = 2)
        
    else
        plot(X3(1:k),Y3(1:k),'b-.',LineWidth = 2)
  
    end
    axis([0 22 -2 6])
    plot(20,0,'o')
    text(19,-0.2,'Target')
    pause(0.000001)

    
    if k ~= length(X3)
        clf
    end
        
end    

% Plot trajectory at 0 deg.
figure
plot(X1,Y1,'m-.',LineWidth = 2)
xlabel('X [m]');
ylabel('Y [m]');
title('Intercept Trajectories at \gamma_f = 0^o')
grid on
grid minor
hold on
plot(20,0,'ro',LineWidth = 5)
axis([0 22 -2 7.5])
text(19,-0.2,'Target')
legend('\gamma_f = 0 deg')

% Plot trajectory at -45 deg.
figure
plot(X2,Y2,'g-.',LineWidth = 2)
xlabel('X [m]');
ylabel('Y [m]');
title('Intercept Trajectories at \gamma_f = -45^o')
grid on
grid minor
hold on
plot(20,0,'ro',LineWidth = 5)
axis([0 22 -2 7.5])
text(19,-0.2,'Target')
legend('\gamma_f = -45 deg')

% Plot trajectory at -90 deg.
figure
plot(X3,Y3,'b-.',LineWidth = 2)
xlabel('X [m]');
ylabel('Y [m]');
title('Intercept Trajectories at \gamma_f = -90^o')
grid on
grid minor
hold on
plot(20,0,'ro',LineWidth = 5)
axis([0 22 -2 7.5])
text(19,-0.2,'Target')
legend('\gamma_f = -90 deg')

% Plot accelration at 0 deg
figure
plot(time_1(2:2018,1),a1(2:2018,1),'m-.',LineWidth = 2)
xlabel('Time [Sec]');
ylabel('Acceleration Command [m/s^2]');
title('Acceleration')
grid on
grid minor
legend('\gamma_f = 0 deg')

% Plot accelration at -45 deg
figure
plot(time_2(2:2120,1),a2(2:2120,1),'g-.',LineWidth = 2)
xlabel('Time [Sec]');
ylabel('Acceleration Command [m/s^2]');
title('Acceleration')
grid on
grid minor
legend('\gamma_f = -45 deg')

% Plot accelration at -90 deg
figure
plot(time_3(2:2401,1),a3(2:2401,1),'b-.',LineWidth = 2)
xlabel('Time [Sec]');
ylabel('Acceleration Command [m/s^2]');
title('Acceleration')
grid on
grid minor
legend('\gamma_f = -90 deg')

% Plot Missile flight angle vs Time
figure
plot(time_1,Gamma_1,'m-.',LineWidth = 2)
xlabel('Time [Sec]');
ylabel('Missile Heading Angle [Deg]');
title('Heading Angle')
grid on
grid minor
legend('\gamma_f = 0 deg')

% Plot Missile flight angle vs Time at -45 deg
figure
plot(time_2,Gamma_2,'g-.',LineWidth = 2)
xlabel('Time [Sec]');
ylabel('Missile Heading Angle [Deg]');
title('Heading Angle')
grid on
grid minor
legend('\gamma_f = -45 deg')

% Plot Missile flight angle vs Time at -90 deg
figure
plot(time_3,Gamma_3,'b-.',LineWidth = 2)
xlabel('Time [Sec]');
ylabel('Missile Heading Angle [Deg]');
title('Heading Angle')
grid on
grid minor
legend('\gamma_f = -90 deg')

% compare all three plots
figure
plot(X1,Y1,'m-.',LineWidth = 2)
xlabel('X [m]');
ylabel('Y [m]');
title('Intercept Trajectories')
grid on
grid minor
hold on
plot(X2,Y2,'g-.',LineWidth = 2)
plot(X3,Y3,'b-.',LineWidth = 2)
plot(20,0,'ro',LineWidth = 5)
text(19,-0.2,'Target')
legend('\gamma_f = 0 deg','\gamma_f = -45 deg','\gamma_f = -90 deg')

% Plot Missile flight angle vs Time
figure
plot(time_1,Gamma_1,'m-.',LineWidth = 2)
xlabel('Time [Sec]');
ylabel('Missile Heading Angle [Deg]');
title('Heading Angle')
grid on
grid minor
hold on
plot(time_2,Gamma_2,'g-.',LineWidth = 2)
plot(time_3,Gamma_3,'b-.',LineWidth = 2)
legend('\gamma_f = 0 deg','\gamma_f = -45 deg','\gamma_f = -90 deg')

% Plot Acceleration with respect to time
figure
plot(time_1(2:2018,1),a1(2:2018,1),'m-.',LineWidth = 2)
xlabel('Time [Sec]');
ylabel('Acceleration Command [m/s^2]');
title('Acceleration')
grid on
grid minor
hold on
plot(time_2(2:2120,1),a2(2:2120,1),'g-.',LineWidth = 2)
plot(time_3(2:2401,1),a3(2:2401,1),'b-.',LineWidth = 2)
legend('\gamma_f = 0 deg','\gamma_f = -45 deg','\gamma_f = -90 deg')

save A4;
