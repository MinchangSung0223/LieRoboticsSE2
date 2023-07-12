function ret=dexpSE2(lambda)
    if lambda(3)==0
        ret = eye(3) + 1/2*ad(lambda);
        return;
    end
    theta = lambda(3);
    ret=eye(3) + (1-cos(theta))/theta^2 *ad(lambda) + (theta-sin(theta))/theta^3*ad(lambda)*ad(lambda);
end
