classdef TwoLinkRobot < Robot
    %TWOLINKROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot
    end
    
    methods
        function obj = TwoLinkRobot()

            G = eye(3);
            obj.addLink([0.0 0]',0,G);
            obj.addLink([0.5 0]',0.5,G);
            obj.addLink([1.5 0]',0.5,G);
            obj.addJoint([0 0]');
            obj.addJoint([1 0]');
            
        end
        

    end
end

