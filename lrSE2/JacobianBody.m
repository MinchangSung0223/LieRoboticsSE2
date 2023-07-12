function [Jb,Jbdot] = JacobianBody(Blist, thetalist,dthetalist)
Jb = Blist;
Jbdot = zeros(size(Blist));
T = eye(3);
JOINTNUM = length(thetalist);
prev_dJidt = Blist(:,JOINTNUM).*dthetalist(JOINTNUM);
for i =  JOINTNUM- 1: -1: 1   
    T = T * expSE2((-1 * Blist(:, i + 1) * thetalist(i + 1)));
    Jbi = Adjoint(T)*Blist(:,i); 
    Jb(:, i) = Jbi;
    Jbdot(:,i) = ad(Jbi)*prev_dJidt;
    prev_dJidt =prev_dJidt+ Jbi.*dthetalist(i);		
end
end

