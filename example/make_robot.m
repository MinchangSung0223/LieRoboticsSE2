close all;
clear;
robot = Robot();
G = eye(3);
robot.addLink([0.0 0]',0,G); % Link 0 com, com to next link, Inertia/mass matrix
robot.addLink([1.0 0]',1.0,G);% Link 1 com, com to next link, Inertia/mass matrix
robot.addLink([3.0 0]',1.0,G);% Link 2 com, com to next link, Inertia/mass matrix
robot.addLink([5.0 0]',1.0,G);% Link 3 com, com to next link, Inertia/mass matrix
robot.addLink([7.0 0]',1.0,G);% Link 4 com, com to next link, Inertia/mass matrix

robot.addJoint([0 0]');% JOINT 1 pos
robot.addJoint([2 0]');% JOINT 2 pos
robot.addJoint([4 0]');% JOINT 3 pos
robot.addJoint([6 0]');% JOINT 4 pos
JOINTNUM = robot.joint_num;
robot.drawRobot(zeros(JOINTNUM,1),1);
axis([-9 9 -9 9])

dt = 0.001; % Sampling Time
Tf = 10 ; % Final Time
N = floor(Tf/dt); % Sample Num
drawCount = 0;
g = [0 -9.8]'; % Gravity 
Ftip = [0 0 0]';
thetalist = zeros(JOINTNUM,1);
dthetalist = zeros(JOINTNUM,1);
ddthetalist = zeros(JOINTNUM,1);
taulist = zeros(JOINTNUM,1);

for t =linspace(0,Tf,N)
    % Forward Kinematics
    T = FKinBody(robot.M,robot.Blist,thetalist);
    [Jb,Jbdot] = JacobianBody(robot.Blist,thetalist,dthetalist);
    
    % Inverse Dynamics
    tau_grav=InverseDynamics(thetalist, dthetalist, ddthetalist, ...
                                   g, Ftip, robot.Mlist, robot.Glist, robot.Slist);
    taulist =tau_grav*0;
    % Forward Dynamics
    ddthetalist = ForwardDynamics(thetalist, dthetalist, taulist, ...
                                       g, Ftip, robot.Mlist, robot.Glist, robot.Slist);
    % Euler Step
    dthetalist = dthetalist + ddthetalist*dt;
    thetalist = thetalist + dthetalist*dt;
    if(drawCount>50 || t==Tf)
        cla;
        robot.drawRobot(thetalist,1);
        title("Time : "+string(t)+"[s]")
        drawnow;
        drawCount=0;
    end
    drawCount=drawCount+1;
end
