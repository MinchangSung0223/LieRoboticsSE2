function taulist = InverseDynamics(thetalist, dthetalist, ddthetalist, ...
                                   g, Ftip, Mlist, Glist, Slist)


n = size(thetalist, 1);
Mi = eye(3);
Ai = zeros(3, n);
AdTi = zeros(3, 3, n + 1);
Vi = zeros(3, n + 1);
Vdi = zeros(3, n + 1);
Vdi(1: 2, 1) = -g;
AdTi(:, :, n + 1) = Adjoint(TransInv(Mlist(:, :, n + 1)));
Fi = Ftip;
taulist = zeros(n, 1);
for i=1: n   
    Mi = Mi * Mlist(:, :, i);
    Ai(:, i) = Adjoint(TransInv(Mi)) * Slist(:, i);

    AdTi(:, :, i) = Adjoint(expSE2((Ai(:, i) ...
                    * -thetalist(i))) * TransInv(Mlist(:, :, i)));

    Vi(:, i + 1) = AdTi(:, :, i) * Vi(:, i) + Ai(:, i) * dthetalist(i);
    Vdi(:, i + 1) = AdTi(:, :, i) * Vdi(:, i) ...
                    + Ai(:, i) * ddthetalist(i) ...
                    + ad(Vi(:, i + 1)) * Ai(:, i) * dthetalist(i);    
end
for i = n: -1: 1
    Fi = AdTi(:, :, i + 1)' * Fi + Glist(:, :, i) * Vdi(:, i + 1) ...
         - ad(Vi(:, i + 1))' * (Glist(:, :, i) * Vi(:, i + 1));


    taulist(i) = Fi' * Ai(:, i);
end
end