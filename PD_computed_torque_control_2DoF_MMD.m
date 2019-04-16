function [doty] = PD_computed_torque_control_2DoF_MMD(M,C,G,torque_dist,Kpd,qd,dqd,ddqd,y)
% Follows the procedure pesented in pa2.4.4 p.185 in Robot Manipulator
% Control - Theory and Practice 2nd ed.
% doty = zeros(4,1);
% qd = desired final position
%  t = time
%  y = [q dq] current q,dq

%  e = qd - q  (1)
% de = dqd- dq (2)
% error_input = [e de];
% dde = ddqd - ddq (3)
% Kpd = [Kp | Kd]

xd = [qd; dqd];
% Define tracking error
% error_input = xd - y; % input to controller
error_input = y; % input to controller

e = error_input(1:2);
de = error_input(3:4);

% Robot Dynamics are given from eq.I:
% M(q)*ddq + C(q,dq)*dq + G(q,dq) + torque_dist = torque (I)
% eq.I is transformed to eq.II:
% M(q)*ddq + N(q,dq) + torque_dist = torque (II)
N = C*y(3:4)+G; % 2x1

%% PD control as in p.188
% For PD control we choose as control function u(t) = -Kp*e - Kd*de
Kp = Kpd(1:2,1:2); % 2x2
Kd = Kpd(1:2,3:4); % 2x2

% Decoupled control unit
u = -Kp*e - Kd*de; % 2x1 % output from controller, input to robot

% Torque applied from robot's actuators
torque = M*(ddqd+u) + N

% Disturbance function w(t) = M^-1*torque_dist
% w = dde + Kd*de + Kp*e; % 2x1
% w(1:2,:) = inv(M)*torque_dist;
w = [1 1]';

% For state vector x = [e de] 4x1 vector
% d(x)/dt = A * x + B * w
I = eye(2);
O = zeros(2);
A = [O I; -Kp -Kd]; 
B = [O; I];
C = eye(4);
D = zeros(4,2);

%% Create ss system
% % t=10;
% % sys = ss(A,B,C,D,t); % PD_computed torque system to steady state
% % ConMax = ctrb(sys.A,sys.B); % SS Controllability matrix
% % NumUnconStates = length(sys.A)-rank(ConMax); % number of uncontrollable states
% % 
% % if NumUnconStates == 0
% %     % System controllable
% %     controllable ='true'
% % else
% %     % System uncontrollable
% %     controllable ='false'
% % end

%% System of differential equations to solve
doty = A*y + B*w;

end