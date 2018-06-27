function [point_grid] = env_to_grid_coordinates(point_env, map_params)
% Convert coordinates from environment to grid map representation.

% void GridMap::environmentToGridCoordinates(geometry_msgs::Point* point) const {
%   // Round down here, because the points here are indices to the grid data
%   // structure (starting from 0).
%   double f1;
%   std::modf((point->x - position_(0)) / resolution_(0), &f1);
%   point->x = f1;
%   std::modf((point->y - position_(1)) / resolution_(1), &f1);
%   point->y = f1;
% }

%point_grid(:,1) = round((point_grid(:,1)/map_params.resolution + map_params.dim_y/2));
%point_grid(:,2) = round((point_grid(:,2)/map_params.resolution + map_params.dim_x/2));

% Round up here for correct indexing to grid map structure.
% (starting from 1)
point_grid(:,1) = ...
floor(round((point_env(:,2) - map_params.position_y) / ...
map_params.resolution, 4)) + 1;
point_grid(:,2) = ...
    floor(round((point_env(:,1) - map_params.position_x) / ...
    map_params.resolution, 4)) + 1;

end