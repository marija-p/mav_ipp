function [entropy_map] = get_map_entropy(grid_map)
% Compute the entropy of a (logodds) occupancy grid map
% Slide 15 from:
% http://ais.informatik.uni-freiburg.de/teaching/ss09/robotics/slides/q_igexplore.pdf

% Compute entropy for each cell.
grid_map = logodds_to_prob(grid_map);
entropy_cells = -(grid_map.*log(grid_map) + (1-grid_map).*log(1-grid_map));

% Sum over map.
entropy_map = nansum(entropy_cells(:));

end

