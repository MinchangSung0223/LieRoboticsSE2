function T=expSE2(lambda)
    theta = lambda(3);
    G = Gmat(theta);
    T = [expSO2(theta) G*[lambda(1) lambda(2)]'; 0 0 1];
end

function ret=Gmat(theta)
    s = sin(theta)/theta;
    c = (1-cos(theta))/theta;
    if theta==0
        ret = eye(2);
        return;
    end
    ret=[s -c; c s];
end