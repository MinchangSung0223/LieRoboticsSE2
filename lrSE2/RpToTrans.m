function T=RpToTrans(R,p)
    T=eye(3);
    T(1:2,1:2)=R;
    T(1,3) = p(1);
    T(2,3) = p(2);
    
end