test_pos = 20:0.01:21;
result_env_grid = zeros(length(test_pos), 3);

for i = 1:length(test_pos)
    pos = [0, test_pos(i), 10];
    result_env_grid(i,2:3) = env_to_grid_coordinates(pos, map_params);
    result_env_grid(i,1) = test_pos(i);
end

test_pos = 410:420;
result_grid_env = zeros(length(test_pos), 3);

for i = 1:length(test_pos)
    pos = [test_pos(i), 1];
    result_grid_env(i,2:3) = grid_to_env_coordinates(pos, map_params);
    result_grid_env(i,1) = test_pos(i);
end