% Initialise ROS comms.
pose_pub = rospublisher('/firefly/command/pose', ...
    rostype.geometry_msgs_PoseStamped);
pose_msg = rosmessage(pose_pub);
odom_sub = rossubscriber('/firefly/ground_truth/odometry');
img_seg_sub = rossubscriber('/firefly/image_seg');
pause(1);

process_img_srv = rossvcclient('/firefly/process_image');
process_img_req = rosmessage(process_img_srv);

% Define static transformations required for co-ordinate conversions.
transforms.T_VSB_CAM = ...      % Vicon sensor body -> camera.
    [0, -1.0000, 0, 0;
    -1.0000, 0, 0, 0;
    0, 0, -1.0000, -0.08;
    0, 0, 0, 1.0000];
transforms.T_MAP_W = eye(4);    % Map -> world

% Environment dimensions [m].
dim_x_env = 200;
dim_y_env = 290;

[matlab_params, planning_params, opt_params, map_params] = ...
    load_params(dim_x_env, dim_y_env);
metrics = initialise_metrics();

if (~exist('ground_truth'))
    load ground_truth.mat
end

% Occupancy grid - 3 layers for 3 classes.
% Initialise with unknown values (probability = 0.5, likelihood = 0).
grid_map = zeros(map_params.dim_y, map_params.dim_x, 3);

% Find cell indices of "interesting" regions based on ground truth.
ground_truth_interesting = ground_truth(2:end-1,2:end-1,planning_params.interesting_class_ind);
interesting_cells_ind = find(ground_truth_interesting == 1);

img_counter = 0;

%% Coverage Planning %%

% Fixed altitude for coverage pattern.
coverage_altitude = 155; % Pattern 1 (halves)
%coverage_altitude = 104.4; % Pattern 2 (thirds)
%coverage_altitude = 39.17; % Pattern 3 (quarters)

coverage_velocity = 2.5;
coverage_acceleration = 10;

boundary = 0;

% Set starting point.
submap_edge_size = get_submap_edge_size_env(coverage_altitude, ...
    planning_params);
point = [-dim_x_env/2+submap_edge_size.x/2 + boundary, ...
    -dim_y_env/2+submap_edge_size.y/2 + boundary, coverage_altitude];

% Create plan (deterministic).
path = point;
i = 0;

while (path(end,1) < (dim_x_env/2 - submap_edge_size.x/2 - boundary))
    
    % Move in y.
    if (mod(i,2) == 0)
        while (path(end,2) < (dim_y_env/2) - submap_edge_size.y/2 - boundary)
            point = path(end,:) + [0, (submap_edge_size.y)/8, 0];
            path = [path; point];
        end
    else
        while (path(end,2) > -dim_y_env/2 + submap_edge_size.y/2 + boundary)
            point = path(end,:) - [0, (submap_edge_size.y)/8, 0];
            path = [path; point];
        end
    end
    path = path(1:end-1, :);
    
    % Move in x.
    point = path(end,:) + [submap_edge_size.x, 0, 0];
    path = [path; point];
    i = i + 1;
    
end

% Remove the last waypoint (out of bounds).
path = path(1:end-1, :);

%% Coverage Execution %%

% Create polynomial trajectory through the control points.
trajectory = plan_path_waypoints(path, coverage_velocity, ...
    coverage_acceleration);

% Sample trajectory to find locations to take measurements at.
[times_meas, points_meas, ~, ~] = ...
    sample_trajectory(trajectory, 1/planning_params.measurement_frequency);

disp(['Total time taken for coverage: ', ...
    num2str(get_trajectory_total_time(trajectory))]);
disp(['Total number of measurements: ', num2str(size(points_meas, 1))]);


% Handle out-of-bounds measurement points.
for i = 1:size(points_meas,1)
    
    %     penalty = max([(points_meas(i,1) + submap_edge_size.x/2 - dim_x_env/2), ...
    %         -(points_meas(i,1) - submap_edge_size.x/2 + dim_x_env/2), ...
    %         (points_meas(i,2) + submap_edge_size.y/2 - dim_y_env/2), ...
    %         -(points_meas(i,2) - submap_edge_size.y/2 + dim_y_env/2), 0.5]);
    %     if (penalty > 0.5)
    %         disp(points_meas(i,:))
    %         error(['Measurement point ', num2str(i), ' is out-of-bounds!'])
    %     end
    if (points_meas(i,1) + submap_edge_size.x/2 > dim_x_env/2)
        points_meas(i,1) = dim_x_env/2 - submap_edge_size.x/2;
    elseif (points_meas(i,1) - submap_edge_size.x/2 < -dim_x_env/2)
        points_meas(i,1) = -dim_x_env/2 + submap_edge_size.x/2;
    end
    if (points_meas(i,2) + submap_edge_size.y/2 > dim_y_env/2)
        points_meas(i,2) = dim_y_env/2 - submap_edge_size.y/2;
    elseif (points_meas(i,2) - submap_edge_size.y/2 < -dim_y_env/2)
        points_meas(i,2) = -dim_y_env/2 + submap_edge_size.y/2;
    end
    
end

% Take measurements along path, updating the grid map.
for i = 1:size(points_meas,1)
    
    % Send the command.
    target_T_MAP_CAM = trvec2tform(points_meas(i,:));
    % We need to make sure the camera is facing down!
    target_T_MAP_CAM(3, 3) = -1;
    target_T_W_VSB = ...
        get_inv_transform(transforms.T_MAP_W)* ...
        target_T_MAP_CAM*get_inv_transform(transforms.T_VSB_CAM);
    target_point = tform2trvec(target_T_W_VSB);
    pose_msg.Pose.Position.X = target_point(1);
    pose_msg.Pose.Position.Y = target_point(2);
    pose_msg.Pose.Position.Z = target_point(3);
    send(pose_pub, pose_msg)
    
    % Go to target measurement point.
    reached_point = false;
    while (~reached_point)
        odom_msg = receive(odom_sub);
        % Ignore orientation for now.
        x_odom_W_VSB = [odom_msg.Pose.Pose.Position.X, ...
            odom_msg.Pose.Pose.Position.Y, odom_msg.Pose.Pose.Position.Z];
        T_W_VSB = trvec2tform(x_odom_W_VSB);
        T_MAP_CAM = transforms.T_MAP_W * T_W_VSB * transforms.T_VSB_CAM;
        x_odom_MAP_CAM = tform2trvec(T_MAP_CAM);
        if (pdist2(points_meas(i,:), x_odom_MAP_CAM) < planning_params.achievement_dist)
            reached_point = true;
        end
    end
    
    pause(0.5);
    
    % Request forward pass through classifier.
    process_img_res = call(process_img_srv, process_img_req);
    odom_msg = receive(odom_sub);

    % Get transform: map -> camera.
    x_odom_W_VSB = [odom_msg.Pose.Pose.Position.X, ...
        odom_msg.Pose.Pose.Position.Y, odom_msg.Pose.Pose.Position.Z];
    quat_odom_W_VSB = [odom_msg.Pose.Pose.Orientation.W, ...
        odom_msg.Pose.Pose.Orientation.X, odom_msg.Pose.Pose.Orientation.Y, ...
        odom_msg.Pose.Pose.Orientation.Z];
    disp(['Taking meas. at: ', num2str(x_odom_W_VSB,4)]);
    
    T_W_VSB = quat2tform(quat_odom_W_VSB);
    T_W_VSB(1:3, 4) = x_odom_W_VSB';
    T_MAP_CAM = transforms.T_MAP_W * T_W_VSB * transforms.T_VSB_CAM;
    x_odom_MAP_CAM = tform2trvec(T_MAP_CAM);
    
    % Get segmented image.
    img_seg = readImage(img_seg_sub.LatestMessage);
    img_counter = img_counter + 1;
    
    % Update the map.
    grid_map = take_measurement_at_point(x_odom_MAP_CAM, img_seg, grid_map, ...
        map_params, planning_params);
    metrics.entropies = [metrics.entropies; ...
        get_map_entropy(grid_map(2:end-1, 2:end-1))];
    metrics.rmses = [metrics.rmses; ...
        get_map_rmse(grid_map(2:end-1,2:end-1), ground_truth(2:end-1,2:end-1))];
    metrics.odoms = [metrics.odoms; odom_msg];
    metrics.grid_maps(:,:,:,img_counter+1) = grid_map;
    
    grid_map_interesting = ...
        grid_map(2:end-1, 2:end-1, planning_params.interesting_class_ind);
    entropy = get_map_entropy(grid_map_interesting(interesting_cells_ind));
    metrics.entropies_interesting = ...
        [metrics.entropies_interesting; entropy];
    rmse = get_map_rmse(grid_map_interesting(interesting_cells_ind), ...
        ground_truth_interesting(interesting_cells_ind));
    metrics.rmses_interesting = [metrics.rmses_interesting; rmse];
    
end