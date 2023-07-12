function drawT(T,linelength,linewidth,alpha)
    p0 = T*[0,0,1]';
    px = T*[linelength,0,1]';
    py = T*[0,linelength,1]';
    plot([p0(1),px(1)],[p0(2),px(2)],'Color',[1,0,0,alpha],"LineWidth",linewidth);hold on;
    plot([p0(1),py(1)],[p0(2),py(2)],'Color',[0,1,0,alpha],"LineWidth",linewidth);
end