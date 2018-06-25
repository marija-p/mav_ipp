function submap_edge_size = get_submap_edge_size(height, ...
    map_params, planning_params)
% Calculates observation window size from height (grid coordinates).

edge_size_x = ceil((1/map_params.resolution)*...
    2*(height*tand(planning_params.sensor_fov_angle_x/2)));
edge_size_x = edge_size_x + mod(edge_size_x-1,2);
edge_size_y = ceil((1/map_params.resolution)*...
    2*(height*tand(planning_params.sensor_fov_angle_y/2)));
edge_size_y = edge_size_y + mod(edge_size_y-1,2);

submap_edge_size.x = edge_size_x;
submap_edge_size.y = edge_size_y;

end

