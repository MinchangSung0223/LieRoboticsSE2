function T = FKinSpace(M, Slist, thetalist)
T = M;
for i = size(thetalist): -1: 1
    T = expSE2(Slist(:, i) * thetalist(i)) * T;
end
end