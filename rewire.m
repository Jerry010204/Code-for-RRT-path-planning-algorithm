function B = rewire(parents, child, oldparent, map, lowestCost, oldId) 
    parent = oldparent;
    id = oldId;
    for k = 1:1:length(parents)
        tempB = parents(k);
        if tempB.cost + dist(child.coord, tempB.coord) < lowestCost
            if noCollision(child.coord, tempB.coord, map)
                parent = tempB;
                lowestCost = tempB.cost + dist(child.coord, tempB.coord);
                id = parents(k).id;
            end
        end
    end
    B = [parent.coord lowestCost id];
end
