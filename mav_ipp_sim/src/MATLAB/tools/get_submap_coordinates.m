function [submap_coordinates] = ...
    get_submap_coordinates(pos_env, submap_edge_size, map_parameters)
% Gets grid coordinates of observed submap of the grid map.

pos_grid = env_to_grid_coordinates(pos_env, map_parameters);

submap_coordinates = [];

xl = pos_grid(2) - floor(submap_edge_size.x/2);
while (xl < 1)
    xl = xl + 1;
end
yd = pos_grid(1) - floor(submap_edge_size.y/2);
while (yd < 1)
    yd = yd + 1;
end
xr = pos_grid(2) + floor(submap_edge_size.x/2);
while (xr > map_parameters.dim_x)
    xr = xr - 1;
end
yu = pos_grid(1) + floor(submap_edge_size.y/2);
while (yu > map_parameters.dim_y)
    yu = yu - 1;
end

submap_coordinates.xl = xl;
submap_coordinates.yd = yd;
submap_coordinates.xr = xr;
submap_coordinates.yu = yu;

end

