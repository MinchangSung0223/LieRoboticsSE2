function Ad=Adjoint(T)
    Ad = eye(3);
    [R,p] = TransToRp(T);
    Ad(1:2,1:2) = R;
    Ad(1,3) = p(2);
    Ad(2,3) = -p(1);
end