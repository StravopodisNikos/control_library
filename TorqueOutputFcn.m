function status = TorqueOutputFcn(t,y,flag,cnt,robot,qd,dqd,ddqd,Kpd)
persistent torque; persistent torques
persistent DCI; persistent DCIs
switch flag
    case 'init'
        torque = [0 0 0 0 0 0]';
        Mm = massMatrix(robot,y(1:6));
        DCI = CalculateDynamicConditioningIndex(Mm,size(Mm,2));
    case '' % compute output variables
        xd = [qd; dqd]; % desired state always constant
        dot_xd = [dqd; ddqd];
        % Define tracking error
        % error_input = [e de];
        error_input = xd-y;
        %  e = qd - q  (1)
        % de = dqd- dq (2)
        e = error_input(1:6);
        de = error_input(7:12);
        
        Mm = massMatrix(robot,y(1:6));
        Vm = velocityProduct(robot,y(1:6),y(7:12));
        Gm = gravityTorque(robot,y(1:6));
        N = Vm+Gm;
        
        DCI = [DCI,CalculateDynamicConditioningIndex(Mm,size(Mm,2))];
        
        Kp = Kpd(1:6,1:6);
        Kd = Kpd(1:6,7:12);
        u = -Kp*e - Kd*de;
        torque = [torque,Mm*(ddqd-u) + N]; % computes torque values
        
    case 'done' % get the data to the workspace.
        assignin('base','torque',torque); 
        assignin('base','DCI',DCI); 
end
status = 0;