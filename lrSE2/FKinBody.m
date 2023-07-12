function T = FKinBody(M, Blist, thetalist)
T = M;
for i = 1: size(thetalist)
    T = T * expSE2(Blist(:, i) * thetalist(i));
end
end
