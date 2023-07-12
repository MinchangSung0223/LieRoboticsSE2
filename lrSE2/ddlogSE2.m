function ret=ddlogSE2(lambda,lambda_dot)
%% time derivative of differential inverse exponential map.(not differential)
%% ddlogSE2 >> d/dt (dexp^{-1}_lambda) 
theta = lambda(3);
theta_dot = lambda_dot(3);
if theta==0
    ret=-1/2*ad(lambda_dot);
    return;
end
ret = -1/2*ad(lambda_dot)+(cot(theta/2)/(2*theta^2) + (cot(theta/2)^2/2 + (1/2))/(2*theta) - 2/theta^3)*theta_dot*ad(lambda)*ad(lambda)+ (1/theta^2 - 1/(2*theta)*cot(theta/2))*(ad(lambda)*ad(lambda_dot)+ad(lambda_dot)*ad(lambda));
end
