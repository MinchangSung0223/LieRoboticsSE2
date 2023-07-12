classdef Robot < handle
    properties
        Slist
        Blist
        com_pos_list = []
        joint_pos_list
        Com_list
        Mlist
        Glist
        joint_num = 0
        link_num = 0
        com_to_joint_length = []
        M
        baseT
        fig;
        fig_num
    end
    
    methods
        function obj = Robot()
            obj.joint_num = 0;
            obj.Glist = zeros(3,3,obj.joint_num);
            obj.M = eye(3);
            obj.fig_num = 1;
            obj.fig= figure(obj.fig_num);
            obj.Com_list(:,:,1) = eye(3);
            obj.baseT = eye(3);
        end
        function setBaseT(obj,baseT)
            obj.baseT = baseT;
        end
        function addLink(obj,com_pos,com_to_joint_length,G)
            obj.com_pos_list = [obj.com_pos_list; reshape(com_pos,1,2)];
            obj.link_num = obj.link_num + 1;
            obj.Glist(:,:,obj.link_num) = G;    
            M_next = [expSO2(0),[com_pos(1); com_pos(2)]; 0 0 1];
            obj.Com_list(:,:,obj.link_num) = M_next;            
            for i = 2:1:size(obj.Com_list,3)
                obj.Mlist(:,:,i-1) = TransInv(obj.Com_list(:,:,i-1))*obj.Com_list(:,:,i);
            end
            obj.com_to_joint_length = [obj.com_to_joint_length,com_to_joint_length];
            if obj.link_num>1
                obj.M = M_next*[1 0 obj.com_to_joint_length(end);0 1 0 ; 0 0 1];
            end
            obj.Mlist(:,:,obj.link_num) = [1 0 obj.com_to_joint_length(end);0 1 0 ; 0 0 1];
            
        end
        
        function addJoint(obj,joint_pos)
            obj.joint_pos_list = [obj.joint_pos_list , reshape(joint_pos,1,2) ];
            obj.Slist = [obj.Slist,[joint_pos(2) -joint_pos(1) 1]'];
            obj.joint_num = obj.joint_num + 1;
        end
        function drawRobot(obj,thetalist,scale)
            
            fig_ = obj.fig;
            line_length = 0.2*scale*5;
            line_width = 2*scale;
            drawT(obj.baseT*eye(3),line_length,line_width,1);
            
            Mi = eye(3);
            Ti = obj.baseT*eye(3);
            com_pos_list = [];
            joint_pos_list= [obj.baseT(1,3),obj.baseT(2,3)];
            for i =1:1:obj.joint_num
                Mi =Mi *obj.Mlist(:,:,i);
                Ai = Adjoint(TransInv(Mi))*obj.Slist(:,i);
                T_i_1_i = obj.Mlist(:,:,i)*expSE2(Ai*thetalist(i));
                Ti = Ti*(T_i_1_i);
                Tjoint = obj.baseT*FKinSpace(Mi*[1 0 obj.com_to_joint_length(end);0 1 0 ; 0 0 1],obj.Slist(:,1:i),thetalist(1:i));
                joint_pos_list=[joint_pos_list;[Tjoint(1,3),Tjoint(2,3)]];
                com_pos_list = [com_pos_list; [Ti(1,3),Ti(2,3)]];
            end
            Teef=obj.baseT*FKinSpace(obj.M,obj.Slist,thetalist);            
            plot(com_pos_list(:,1),com_pos_list(:,2),'ko',"MarkerSize",line_width*2);
            plot(com_pos_list(:,1),com_pos_list(:,2),'k+',"MarkerSize",line_width*2);
            plot(joint_pos_list(:,1),joint_pos_list(:,2),'k-',"LineWidth",line_width*5,"Color",[0 0 0 0.2]);
            plot(joint_pos_list(1:length(joint_pos_list)-1,1),joint_pos_list(1:length(joint_pos_list)-1,2),'k.',"MarkerSize",line_width*10);
            plot(joint_pos_list(1:length(joint_pos_list)-1,1),joint_pos_list(1:length(joint_pos_list)-1,2),'y.',"MarkerSize",line_width*5);
            drawT(Teef,line_length,line_width,1);
            daspect([1,1,1]);
            maxVal = norm([obj.M(1,3),obj.M(2,3)])*1.0;
            axis([-maxVal maxVal -maxVal maxVal]);
            grid on;
        end        
    end
end
