% For the use of HKU MECH3433 Robotics, drones and autonomous ground vehicles.
% n1----->n2
function nc = noCollision(n1, n2, map)
    d = norm(n2-n1);
    if d == 0
        nc = 0; return;
    end
    dir  = (n2-n1)/d;
    delta = 1.25;
    for t = 0:delta:(d+1.25)
        temp = n1 + dir*t;
%         fprintf('dir [%f %f]', dir(1), dir(2))
        idx= ceil((100-temp(2))/1.25);
        idy = ceil(temp(1)/1.25);
%         fprintf('[%d, %d] [%f, %f] [%f, %f], [%f, %f]NO Collision \n', idx, idy,  temp(1), temp(2), n1(1), n1(2), n2(1), n2(2));
        if idx >80 idx = 80;end
        if idx <1 idx = 1;end
        if idy > 99 idy = 99;end
        if idy < 1 idy = 1;end
        for i = 0:3
            for j = 0:3
                if idx+i>80 || idx+i<1 || idy+j>99 || idx+j<1
                    continue
                end
                if map(idx+i, idy+j) == 1
                    nc = 0; return;
                end
            end
        end
%         fprintf('[%d, %d] [%f, %f] NO Collision \n', idx, idy,  temp(1), temp(2));
    end
    nc = 1;      
end
