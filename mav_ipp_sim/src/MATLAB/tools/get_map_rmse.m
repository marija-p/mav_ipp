function rmse = get_map_rmse(grid_map, ground_truth_map)
% Computes the RMSE of a map wrt. ground truth.

grid_map = logodds_to_prob(grid_map);

se = sum(sum(sum((grid_map(2:end-1,2:end-1,:) - ...
    ground_truth_map(2:end-1,2:end-1,:).^2))));
mse = se / numel(ground_truth_map);
rmse = sqrt(mse);

end

