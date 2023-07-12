function [X_traj,V_traj,Vdot_traj]=LieScrewTrajectory(X_start,X_end,V_start,V_end,Vdot_start,Vdot_end,Tf,N)
timegap = Tf / (N - 1);
X_traj = cell(1, N);
V_traj =  cell(1, N);
Vdot_traj =  cell(1, N);

lambda0 = [0 0 0]';
lambdaT = logSE2(TransInv(X_start)*X_end);

lambda_dot0 = V_start;
lambda_ddot0 =Vdot_start;
lambda_dotT = dlogSE2(-lambdaT)*V_end;
lambda_ddotT = dlogSE2(-lambdaT)*Vdot_end+ddlogSE2(-lambdaT,-lambda_dotT)*V_end;

for i = 1: N
    t = timegap * (i - 1);
    lambda_t = zeros(3,1);
    lambda_dot_t = zeros(3,1);
    lambda_ddot_t = zeros(3,1);
    for j =1:1:3
        [lambda_t_,lambda_dot_t_,lambda_ddot_t_]=QuinticTimeScalingKinematics(lambda0(j),lambdaT(j),lambda_dot0(j),lambda_dotT(j),lambda_ddot0(j),lambda_ddotT(j),Tf,t);
        lambda_t(j) = lambda_t_;
        lambda_dot_t(j) = lambda_dot_t_;
        lambda_ddot_t(j) = lambda_ddot_t_;
    end
    V = dexpSE2(-lambda_t)*lambda_dot_t;    
    Vdot = dexpSE2(-lambda_t)*lambda_ddot_t+ ddexpSE2(-lambda_t,-lambda_dot_t)*lambda_dot_t;
    T = X_start*expSE2(lambda_t);
    X_traj{i} = T;
    V_traj{i}= V;
    Vdot_traj{i}= Vdot;
end

end


function [s,ds,dds]=QuinticTimeScalingKinematics(s0,sT,ds0,dsT,dds0,ddsT,Tf,t)
    x(1) = s0;
    x(2) = ds0;
    x(3) = dds0/2.0;
    x(4) = -(10*s0 - 10*sT + 2*Tf*(3*ds0 + 2*dsT) + (Tf^2*(3*dds0 - ddsT))/2)/Tf^3;
    x(5) = (((3*dds0)/2 - ddsT)*Tf^2 + (8*ds0 + 7*dsT)*Tf + 15*s0 - 15*sT)/Tf^4;
    x(6) = -(6*s0 - 6*sT + (Tf^2*(dds0 - ddsT))/2 + 3*Tf*(ds0 + dsT))/Tf^5;
    s = x(1)+x(2)*t+x(3)*t^2+x(4)*t^3+x(5)*t^4+x(6)*t^5;
    ds = x(2)+2*x(3)*t+3*x(4)*t^2+4*x(5)*t^3+5*x(6)*t^4;
    dds = 2*x(3)+2*3*x(4)*t+3*4*x(5)*t^2+4*5*x(6)*t^3;
end