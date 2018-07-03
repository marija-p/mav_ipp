function obj = compute_objective(control_points, grid_map, map_params, ...
    planning_params)
% Calculates the expected informative objective for a polynomial path.
% ---
% Inputs:
% control_points: list of waypoints defining the polynomial
% grid_map: current occupancy grid map
% ---
% Output:
% obj: informative objective value (to be minimized)
% ---
% M Popovic 2018
%

dim_x_env = map_params.dim_x*map_params.resolution;
dim_y_env = map_params.dim_y*map_params.resolution;

% Create polynomial path through the control points.
trajectory = plan_path_waypoints(control_points, ...
    planning_params.max_vel, planning_params.max_acc);

% Sample trajectory to find locations to take measurements at.
[~, points_meas, ~, ~] = sample_trajectory(trajectory, ...
    1/planning_params.measurement_frequency);

entropy_i = get_map_entropy(grid_map);

% Discard path if it is too long.
if (size(points_meas,1) > 10)
    obj = Inf;
    return;
end

% if (any(points_meas(:,1) > dim_x_env/2) || ...
%         any(points_meas(:,2) > dim_y_env/2) || ...
%         any(points_meas(:,1) < -dim_x_env/2) || ...
%         any(points_meas(:,2) < -dim_y_env/2) || ...
%         any(points_meas(:,3) < planning_params.min_height) || ...
%         any(points_meas(:,3) > planning_params.max_height))
%     obj = Inf;
%     return;
% end

% Predict measurements along the path.
for i = 1:size(points_meas,1)
    % Discard out-of-bounds solutions.
    submap_edge_size_env = get_submap_edge_size_env(points_meas(i,3), planning_params);
    if (points_meas(i,1) + submap_edge_size_env.x/2) >= dim_x_env/2 || ...
            (points_meas(i,1) - submap_edge_size_env.x/2) < -dim_x_env/2 || ...
            (points_meas(i,2) + submap_edge_size_env.y/2) >= dim_y_env/2 || ...
            (points_meas(i,2) - submap_edge_size_env.y/2) < -dim_y_env/2 || ...
            points_meas(i,3) < planning_params.min_height || ...
            points_meas(i,3) > planning_params.max_height
        obj = Inf;
        return;
    end
    % Discard crappy solutions.
    try
        grid_map = predict_map_update(points_meas(i,:), grid_map, ...
            map_params, planning_params);
    catch
        obj = Inf;
        return;
    end
end

entropy_f = get_map_entropy(grid_map);

% Formulate objective.
gain = entropy_i - entropy_f;
cost = max(get_trajectory_total_time(trajectory), ...
    1/planning_params.measurement_frequency);
obj = -gain/cost;

%disp(['Measurements = ', num2str(i)])
%disp(['Gain = ', num2str(gain)])
%disp(['Cost = ', num2str(cost)])
%disp(['Objective = ', num2str(obj)])

end