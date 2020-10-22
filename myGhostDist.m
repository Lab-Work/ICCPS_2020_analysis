function [ghostDist,egoDist] = myGhostDist(bag)
    ghostDist_bag = select(bag,'Topic','/ghost_dist_traveled');
    ghostDist = ghostDist_bag.timeseries;
    egoDist_bag = select(bag,'Topic','/ego_dist_traveled');
    egoDist = egoDist_bag.timeseries;
end
