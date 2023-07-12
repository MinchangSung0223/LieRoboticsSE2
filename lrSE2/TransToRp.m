function [R,p]=TransToRp(T)
    R =T(1:2,1:2);
    p = T(1:2,3);
end