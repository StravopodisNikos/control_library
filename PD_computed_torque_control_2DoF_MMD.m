function xdot = PD_computed_torque_control_2DoF_MMD(t,x,M,C,G,torque_dist,Kpd,qd)
% Follows the procedure pesented in pa2.4.4 p.185 in Robot Manipulator
% Control - Theory and Practice 2nd ed.

% qd = desired final position
%  t = time
%  x = [e de] state vector 4x1 for PD control
%  e = qd - q  (1)
% de = dqd- dq (2)
% dde = ddqd - ddq (3)
% Kpd = [Kp | Kd]

xdot = zeros(4,1);

% Robot Dynamics are given from eq.I:
% M(q)*ddq + C(q,dq)*dq + G(q,dq) + torque_dist = torque (I)

% eq.I is transformed to eq.II:
% M(q)*ddq + N(q,dq) + torque_dist = torque (II)
N = C*x(3:4)+G; % 2x1 vector

% Define tracking error
e = qd - x(1:2); % 2x1
xdot(1:2) = e;

%% Feedback Linearization
% Solving for ddq in eq.II yields to:
% ddq = M^-1*(torque - torque_dist - N(q,dq))
% ddq = M^-1*(torque - N(q,dq)) - M^-1* torque_dist  (III)

% From eq. (III) & (3) we get:
% dde = ddqd - M^-1*( torque - N(q,dq)) + M^-1*torque (IV)
%               ==================ddq================

% Since state vector is: x = [e de], 
% then dx = d[e de]/dt = [de dde] a non-linear system of 2 equations
% dx = [ dqd - dq; ddqd - M^-1*( torque - N(q,dq)) + M^-1*torque ] (V)

%% PD control as in p.188
% We linearize eq.V using feedback linearization and a PD controller
% For PD control we choose as control function u(t) = -Kp*e - Kd*de
Kp = Kpd(1:2,1:2); % 2x2
Kd = Kpd(1:2,3:4); % 2x2

% Decoupled control unit
u = -Kp*e - Kd*de; % 2x1

% Disturbance function w(t) = M^-1*torque_dist
w = dde + Kd*de + Kp*e; % 2x1
% For state vector x = [e de] 4x1 vector
% d(x)/dt = A * x + B * w
I = eye(2);
O = zeros(2);
A = [O I; -Kp -Kd]; 
B = [O I]';







end