% For the use of HKU MECH3433 Robotics, drones and autonomous ground vehicles. 
% 
% RRT* algorithm in 2D with collision avoidance.
% nodes:    Contains list of all explored nodes. Each node contains its
%           coordinates, cost to reach and its parent.
% 
% Brief description of algorithm: 
% 1. Pick a random node q_rand.
% 2. Find the closest node q_near from explored nodes to branch out from, towards
%    q_rand.
% 3. Steer from q_near towards q_rand: interpolate if node is too far away, reach
%    q_new. Check that obstacle is not hit.
% 4. Update cost of reaching q_new from q_near, treat it as Cmin. For now,
%    q_near acts as the parent node of q_new.
% 5. From the list of 'visited' nodes, check for nearest neighbors with a 
%    given radius, insert in a list q_nearest.
% 6. In all members of q_nearest, check if q_new can be reached from a
%    different parent node with cost lower than Cmin, and without colliding
%    with the obstacle. Select the node that results in the least cost and 
%    update the parent of q_new.
% 7. Add q_new to node list.
% 8. Continue until maximum number of iteration is reached.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     Search for keyword "step" to locate where you
%     are required to do the programming.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clearvars
close all

x_max = 120;
y_max = 100;
EPS = 10;
numIteration = 1800;    

q_start.coord = [95.99, 57.65];
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [10 40];
q_goal.cost = + inf;

reach_g = 0;
nodes(1) = q_start;
figure(1)
axis([0 x_max 0 y_max])
load('obstacle.mat', 'obstacle');
load('map.mat', 'map');

% Show start point
rectangle('Position',[95.99, 57.65, 1, 1],'FaceColor',[0 1 0]) 
% Show goal point
rectangle('Position',[10, 40, 2, 2],'FaceColor',[1 0. 0.])
for i = 1 : size(obstacle, 1)
%     fprintf('job done \n');
    rectangle('Position',obstacle(i,:),'FaceColor',[0 .5 .5])
end
hold on

for i = 1:1:numIteration
    fprintf('[NO %d]Selecting a new node...\n', i);
    if rand(1) > 0.9
        q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
        plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410])
    else
        q_rand = q_goal.coord;
        plot(q_goal.coord(1), q_goal.coord(2), 'x', 'Color',  [0 0.4470 0.7410])
    end
    
%     disp('One shot connection');
    [eId, status] = oneShot(nodes, q_goal.coord, map);
    if status == 1
        if q_goal.cost > nodes(eId).cost + norm(q_goal.coord - nodes(eId).coord)
            q_goal.cost = nodes(eId).cost + norm(q_goal.coord - nodes(eId).coord);
            q_goal.parent = eId;
            line([nodes(eId).coord(1), q_goal.coord(1)], [nodes(eId).coord(2), q_goal.coord(2)], 'Color', 'k', 'LineWidth', 2);
            drawnow
            hold on
        end
        reach_g = 1;
    end
    
    
    % Pick the closest node from existing list to branch out from
    ndist = []; 
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);
   
%     disp('Steer the closest point towarder the random generated node');
    % Step 1: Go the "steer.m" script to implement the function of steer.
    q_new.coord = steer(q_rand, q_near, val, EPS);
    if noCollision(q_near.coord, q_new.coord, map)
        q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;
     
%         disp('Find all nodes in radisu r');
        % Within a radius of r, find all existing nodes
        q_nearest = []; 
        r = 20;
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            if noCollision(q_new.coord, nodes(j).coord, map) && dist(nodes(j).coord, q_new.coord) <= r
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                q_nearest(neighbor_count).id = j;%%%ed
                neighbor_count = neighbor_count+1;
            end
        end
        
        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;
        
%         disp('Rewire the search tree');
        % Step 2: Iterate through all nearest neighbors to find alternate lower
        % cost paths.
        %parentcost = rewire(q_nearest, q_new, q_near, map, C_min);
        for i = 1:1:length(nodes)
            if q_near.coord == nodes(i).coord
                oldParent_Id = i;
            end
        end
        parentCoordCostId = rewire(q_nearest, q_new, q_near, map, C_min, oldParent_Id);
        
        % Step 3: Update parent to least cost-from node
        parentCoord = parentCoordCostId(1:2);
        q_new.cost = parentCoordCostId(3);
        q_new.parent = parentCoordCostId(4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
        drawnow
        hold on
        
        % Append to nodes
        nodes = [nodes q_new];
    end
end

if reach_g == 0
    error('[NO RESULT]： RRT Failed; Please increase the number of nodes.');
end


D = [];
for j = 1:1:length(nodes)
    tmpdist = dist(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
% [val, idx] = min(D);
% q_final = nodes(idx);
% q_goal.parent = idx;
path = [];
q_end = q_goal;
path = [q_end, path];

while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);

    hold on
    q_end = nodes(start);
    path = [nodes(start), path];
end

% If you cannot write in the file, please change the work space of the matlab
save('RRT_Star/2D/minimum-snap/path.mat', 'path')
% save('RRT_Star/2D/minimum-snap/path.mat', 'path')

