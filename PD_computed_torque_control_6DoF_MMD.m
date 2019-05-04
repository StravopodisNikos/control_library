function [doty] = PD_computed_torque_control_6DoF_MMD(robot,torque_dist,Kpd,qd,dqd,ddqd,y)
% Follows the procedure pesented in p.188-189 in Robot Manipulator
% Control - Theory and Practice 2nd ed.
y;
% To be implemented:
% 1.Inner Loop for dynamic properties is implemented, takes into account
%   changes of configuration
% 2.Kp,Kd parameters are manually controlled for critical damping
% 3. torque is calculated iteratively through dynamic equations
% 4. disturbance function is not physical

% qd = desired final joint position-velocity
%  t = time
%  y = [q dq] current q,dq
% Kpd = [Kp | Kd]


% Define desired final state
xd = [qd; dqd]; % desired state always constant
dot_xd = [dqd; ddqd];
% Define tracking error
% error_input = [e de];
error_input = xd-y;
%  e = qd - q  (1)
% de = dqd- dq (2)
e = error_input(1:6);
de = error_input(7:12);

% Robot Dynamics are given from eq.I:
% M(q)*ddq + C(q,dq)*dq + G(q,dq) + torque_dist = torque (I)
% M(q)*ddq + V(q,dq) + G(q,dq) + torque_dist = torque (I)
%% Matlab dynamics first
Mm = massMatrix(robot,y(1:6));
Vm = velocityProduct(robot,y(1:6),y(7:12));
Gm = gravityTorque(robot,y(1:6));
%% My POEs
% [Jbsli_ref,Jssli_ref,Jbsli_POE_ref,gsli_ref] = calculate_CoM_BodyJacobians_6DoF_MMD(y(1:6),xi_ai,Pi_ref,gsli0);
% [M_POE,V_POE,N_POE,Mm,Vm,Nm,JointTorque_POE,JointTorqM] = InverseDynamicProblemPOE_6DoF_MMD(robot,y(1:6),y(7:12),dy(7:12),M0b_CoM,M0s_CoM,Jssli_ref,Jbsli_ref,xi_ai, Pi_ref, gsli0);

% eq.I is transformed to eq.II:
% M(q)*ddq + N(q,dq) + torque_dist = torque (II)
N = Vm+Gm; % 6x1

%% PD control as in p.188
% For PD control we choose as control function u(t) = -Kp*e - Kd*de
Kp = Kpd(1:6,1:6); % 6x6
Kd = Kpd(1:6,7:12); % 6x6

% Decoupled control unit
u = -Kp*e - Kd*de; % 2x1 % output from controller, input to robot

% Torque applied from robot's actuators
torque = Mm*(ddqd-u) + N; % "should be avoided" p.188 Robot Manipulator Control

% Disturbance function w(t) = M^-1*torque_dist
% w = dde + Kd*de + Kp*e; % 2x1
w = inv(Mm)*torque_dist; % inv is super high!!!
% w = [1 1 1 1 1 1]';

% For state vector x = [e de] 4x1 vector
% d(x)/dt = A * x + B * w
I = eye(6);
O = zeros(6);
A = [O I; -Kp -Kd]; 
B = [O; I];
% C = eye(6);
% D = zeros(6,2);

%% Evaluate if ss system is controllable
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
dot_error_input = A*error_input + B*w;
doty = dot_xd - dot_error_input;

end