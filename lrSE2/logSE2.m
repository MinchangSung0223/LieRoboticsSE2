function lambda=logSE2(T)
    lambda=[0;0;0];
    lambda(3) = atan2(T(2,1),T(1,1));
    theta = lambda(3);
    invG = invGmat(theta);
    val = invG*[T(1,3); T(2,3)];
    lambda(1) = val(1);
    lambda(2) = val(2);
    
end
function ret=invGmat(theta)
    a = theta/2*cot(theta/2);
    b = 1/2*theta;
    if theta ==0 
        ret = eye(2);
        return;
    end
    ret=[a b; -b a];
end