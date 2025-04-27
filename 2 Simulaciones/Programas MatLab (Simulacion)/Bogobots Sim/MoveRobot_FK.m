function [pts,T] = MoveRobot_FK (L,q)
    pts = [];
    T = [];
    for i=1:size(q,1)
        qFK = q(i,:);
        [p,t]=FK_robot(L,qFK);
        T = [T;t];
        pts = [pts; p];
    end
end