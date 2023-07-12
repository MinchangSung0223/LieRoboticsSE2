function ret=dlogSE2(lambda)
    theta = lambda(3);
    if theta==0
        ret = eye(3) -1/2*ad(lambda);
        return;
    end
    ret = eye(3)-1/2*ad(lambda)+(1/theta^2 - 1/(2*theta)*cot(theta/2))*ad(lambda)*ad(lambda);
end
