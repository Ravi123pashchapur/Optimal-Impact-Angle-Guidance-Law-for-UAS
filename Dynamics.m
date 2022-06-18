function missile_states_update = Dynamics(missile_states,AM)
global VM1 DT

MX = missile_states(1);
MY = missile_states(2);
VMX = missile_states(3);
VMY = missile_states(4);
AMX = missile_states(5);
AMY = missile_states(6);
GAM_M = missile_states(7);

GAM_M = wrapToPi(GAM_M + AM/VM1*DT);
AMX = - AM*sin(GAM_M);
AMY =   AM*cos(GAM_M);
VMX =   VM1*cos(GAM_M);
VMY =   VM1*sin(GAM_M);
MX  =   MX + VMX*DT;
MY  =   MY + VMY*DT;

missile_states_update = [MX MY VMX VMY AMX AMY GAM_M];