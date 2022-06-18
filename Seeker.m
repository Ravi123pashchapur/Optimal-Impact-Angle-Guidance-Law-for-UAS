function [R1,SIG1,GAM1,SIGR1] = Seeker(target_states,missile_states)
global VM1

sig = (atan2((target_states(2)-missile_states(2)),(target_states(1)-missile_states(1))));
x_m  = cos(sig)*(missile_states(1));
y_m = cos(sig)*(missile_states(2));
x_t = cos(sig)*(target_states(1));
y_t = cos(sig)*(target_states(2));
vx_m = cos(sig)*(missile_states(3));
vy_m = cos(sig)*(missile_states(4));

% Relative distance R
R1 = sqrt((x_t-x_m)^2 + (y_t-y_m)^2);

% LOS from missile to target
SIG1 = (atan2(y_t-y_m,x_t-x_m));

% FPA of missile
GAM1 = (atan2(vy_m,vx_m));

% LOS rate
SIGR1 = (-VM1*sin(GAM1-SIG1))/R1;
end
