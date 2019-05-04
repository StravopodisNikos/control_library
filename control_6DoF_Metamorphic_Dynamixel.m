function [tvec,dy] = control_6DoF_Metamorphic_Dynamixel(robot,qi,qd,dqd,ddqd,cnt)
dt = 0.17; % [s]
tspan = [0 0+dt];
% t = 0.6:0.1:1.5;
icond = vertcat(qi,[0 0 0 0 0 0]');
torque_dist = [0.001 0.001 0.001 0.001 0.001 0.001]'; % [Nm] Chosen arbitrary!!!

zeta = 1;
wn = 100;
kp = [wn^2 wn^2 wn^2 wn^2 wn^2 wn^2]';
KP = diag(kp);
kd = [2*zeta*wn 2*zeta*wn 2*zeta*wn 2*zeta*wn 2*zeta*wn 2*zeta*wn]';
KD = diag(kd);
KPD = horzcat(KP,KD); % 6x12
odeFcn = @(t,y) PD_computed_torque_control_6DoF_MMD(robot,torque_dist,KPD,qd,dqd,ddqd,y);
options = odeset('OutputFcn',@(t,y,flag) TorqueOutputFcn(t,y,flag,cnt,robot,qd,dqd,ddqd,KPD));
[tvec,dy] = ode113(odeFcn,tspan,icond, options);
end