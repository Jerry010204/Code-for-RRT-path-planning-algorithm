% For the use of HKU MECH3433 Robotics, drones and autonomous ground vehicles. 
function [eId, status] = oneShot(nodes, goal, map)
   % Pick the closest node from existing list to connect with goal
    eId = length(nodes);
    q_test = nodes(eId);
%     fprintf('Testing [%f, %f], [%f, %f]', q_test.coord(1), q_test.coord(2), goal(1), goal(2))
    if noCollision(q_test.coord, goal, map)
        status = 1;
    else
        status = 0;
    end
end
