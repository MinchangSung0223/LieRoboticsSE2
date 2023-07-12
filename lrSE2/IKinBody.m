function [thetalist, success] = IKinBody(Blist, M, T, thetalist0, eomg, ev)
thetalist = thetalist0;
i = 0;
maxiterations = 20;
Vb = logSE2(TransInv(FKinBody(M, Blist, thetalist)) * T);
err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
while err && i < maxiterations
    [Jb,Jbdot]=JacobianBody(Blist, thetalist,zeros(size(thetalist)));
    thetalist = thetalist + pinv(Jb) * Vb;
    i = i + 1;
    Vb = logSE2(TransInv(FKinBody(M, Blist, thetalist)) * T);
    err = norm(Vb(3)) > eomg || norm(Vb(1: 2)) > ev;
end
success = ~ err;
end