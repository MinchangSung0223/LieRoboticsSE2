function ret = ad(V)
    ret = [0 -V(3) V(2) ;...
           V(3) 0 -V(1);...
            0 0 0];
end
