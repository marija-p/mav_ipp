function grid_map = take_measurement_at_point(pos, img_seg, grid_map, ...
    map_params, planning_params)

% Compute current FoV (camera footprint).
submap_edge_size_env = get_submap_edge_size_env(pos(3), planning_params);
submap_coordinates_env.yd = pos(2) - submap_edge_size_env.y/2;
submap_coordinates_env.yu = pos(2) + submap_edge_size_env.y/2;
submap_coordinates_env.xl = pos(1) - submap_edge_size_env.x/2;
submap_coordinates_env.xr = pos(1) + submap_edge_size_env.x/2;

% Compute the number of image pixels to trim based on the grid.
submap_coordinates_env_trimmed.yd = ...
    map_params.resolution*ceil(submap_coordinates_env.yd/map_params.resolution);
submap_coordinates_env_trimmed.yu = ...
    map_params.resolution*floor(submap_coordinates_env.yu/map_params.resolution);
submap_coordinates_env_trimmed.xl = ...
    map_params.resolution*ceil(submap_coordinates_env.xl/map_params.resolution);
submap_coordinates_env_trimmed.xr = ...
    map_params.resolution*floor(submap_coordinates_env.xr/map_params.resolution);

num_trim_pixels.left = ...
    abs(submap_coordinates_env.xl - submap_coordinates_env_trimmed.xl)* ...
    (size(img_seg,1)/submap_edge_size_env.x);
num_trim_pixels.right = ...
    abs(submap_coordinates_env.xr - submap_coordinates_env_trimmed.xr)* ...
    (size(img_seg,1)/submap_edge_size_env.x);
num_trim_pixels.up = ...
    abs(submap_coordinates_env.yu - submap_coordinates_env_trimmed.yu)* ...
    (size(img_seg,2)/submap_edge_size_env.y);
num_trim_pixels.down = ...
    abs(submap_coordinates_env.yd - submap_coordinates_env_trimmed.yd)* ...
    (size(img_seg,2)/submap_edge_size_env.y);
num_trim_pixels.left = ceil(num_trim_pixels.left);
num_trim_pixels.right = ceil(num_trim_pixels.right);
num_trim_pixels.up = ceil(num_trim_pixels.up);
num_trim_pixels.down = ceil(num_trim_pixels.down);

% Trim the image to fit on the grid cells.
img_seg_trimmed = img_seg(1+num_trim_pixels.left:end-num_trim_pixels.right, ...
    1+num_trim_pixels.down:end-num_trim_pixels.up, :);

% Compute the number of grid cells to update.
pos1 = [submap_coordinates_env.xl, submap_coordinates_env.yd, pos(3)];
pos1 = env_to_grid_coordinates(pos1, map_params);
submap_coordinates.xl = pos1(2) + 1;
submap_coordinates.yd = pos1(1) + 1;
pos1 = [submap_coordinates_env.xr, submap_coordinates_env.yu, pos(3)];
pos1 = env_to_grid_coordinates(pos1, map_params);
submap_coordinates.xr = pos1(2) - 1;
submap_coordinates.yu = pos1(1) - 1;

%% TODO: Extract submap for fusion.
submap = imresize(img_seg_trimmed, ...
    [submap_coordinates.xr - submap_coordinates.xl + 1, ...
    submap_coordinates.yu - submap_coordinates.yd + 1]);
  
% Update grid map.
grid_map = ...
    update_map(pos, submap, grid_map, submap_coordinates_env, planning_params);

end