function ret=ddexpSE2(lambda,lambda_dot)
%% time derivative of differential inverse exponential map.(not differential)
%% ddexpSE2 >> d/dt (dexp_lambda) 

    theta = lambda(3);
    theta_dot = lambda_dot(3);

    Gamma_1 = (1-cos(theta))/theta^2;
    Gamma_1_dot = (sin(theta)/theta^2 + (2*(cos(theta) - 1))/theta^3)*theta_dot;
    Gamma_1_0 = 1/2;
    Gamma_1_dot_0 = 0;

    Gamma_2 = (theta-sin(theta))/theta^3;
    Gamma_2_0 = 1/6;
    Gamma_2_dot = (- (3*(theta - sin(theta)))/theta^4 - (cos(theta) - 1)/theta^3)*theta_dot;
    Gamma_2_dot_0 = 0;
   if theta ==0
       ret = Gamma_1_0*ad(lambda_dot) + 1/6*(ad(lambda_dot)*ad(lambda)+ad(lambda)*ad(lambda_dot));
       return;
   end
   ret = (sin(theta)/theta^2 - 2*(1-cos(theta))/theta^3)*theta_dot*ad(lambda)+...
        (1-cos(theta))/theta^2*ad(lambda_dot) +...
        ((1-cos(theta))/theta^3 - 3*(theta-sin(theta))/theta^4 )*theta_dot*ad(lambda)*ad(lambda) +...
        (theta-sin(theta))/theta^3*(ad(lambda)*ad(lambda_dot)+ad(lambda_dot)*ad(lambda));
end
