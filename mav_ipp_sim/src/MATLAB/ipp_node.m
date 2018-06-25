% Initialise ROS comms.
pose_pub = rospublisher('/firefly/command/pose', ...
    rostype.geometry_msgs_PoseStamped);
pose_msg = rosmessage(pose_pub);

odom_sub = rossubscriber('/firefly/ground_truth/odometry');
img_seg_sub = rossubscriber('/firefly/image_seg');

process_img_srv = rossvcclient('/firefly/process_image');
process_img_req = rosmessage(process_img_srv);

% Define static transformations required for co-ordinate conversions.
transforms.T_VSB_CAM = ...              % Vicon sensor body -> camera.
    [0, -1.0000, 0, 0;
    -1.0000, 0, 0, 0;
    0, 0, -1.0000, -0.08;
    0, 0, 0, 1.0000];
transforms.T_MAP_W = eye(4);    % Map -> world
 
% Environment dimensions [m].
dim_x_env = 200;
dim_y_env = 295;

[matlab_params, planning_params, opt_params, map_params] = ...
    load_params(dim_x_env, dim_y_env);

% First measurement location.
point_init = [0, 0, 50];
% Multi-resolution lattice.
lattice = create_lattice(map_params, planning_params);
% Occupancy grid - 3 layers for 3 classes.
% Initialise with unknown values (probability = 0.5, likelihood = 0).
grid_map = zeros(map_params.dim_y, map_params.dim_x, 3);

budget_spent = 0;
time_elapsed = 0;
point_prev = point_init;

%% Planning-Execution Loop %%

while (true)
    
    %% Planning %%
    
    %% STEP 1. Grid search on the lattice.
    path = search_lattice(point_prev, lattice, grid_map, map_params, ...
        planning_params);
    obj = compute_objective(path, grid_map, map_params, planning_params);
    disp(['Objective before optimization: ', num2str(obj)]);

    %% STEP 2. Path optimization.
    if (strcmp(opt_params.opt_method, 'cmaes'))
        path_optimized = optimize_with_cmaes(path, grid_map, map_params, ...
            planning_params, opt_params);
            %obj = compute_objective(path_optimized, grid_map, map_params, planning_params);
            %disp(['Objective after optimization: ', num2str(obj)]);
    elseif (strcmp(opt_params.opt_method, 'fmc'))
        path_optimized = optimize_with_fmc(path, grid_map, map_params, ...
            planning_params);
    elseif (strcmp(opt_params.opt_method, 'bo'))
        path_optimized = optimize_with_bo(path, grid_map, map_params, ...
            planning_params);
    else
        path_optimized = path;
    end

    %% Plan Execution %%
    % Create polynomial trajectory through the control points.
    trajectory = ...
        plan_path_waypoints(path_optimized, planning_params.max_vel, ...
        planning_params.max_acc);

    % Sample trajectory to find locations to take measurements at.
    [times_meas, points_meas, ~, ~] = ...
        sample_trajectory(trajectory, 1/planning_params.measurement_frequency);
    
    disp(points_meas)
    
    % Take measurements along path, updating the grid map.
    for i = 1:size(points_meas,1)
        
        % Budget has been spent.
        if ((time_elapsed + times_meas(i)) > planning_parameters.time_budget)
            points_meas = points_meas(1:i-1,:);
            times_meas = times_meas(1:i-1);
            budget_spent = 1;
            break;
        end
        
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
            if (pdist2(points_meas(i,:), x_odom_MAP_CAM) < achievement_dist)
                reached_point = true;
            end
        end
        
        pause(0.5);

        % Request forward pass through classifier.
        process_img_srv = call(process_img_srv, process_img_req, 'Timeout', 100);
        
        % Make sure delay between image/odom messages is small enough.
        delay = 100;
        while (abs(delay) > 0.07)
            img_seg_msg = receive(img_seg_sub);
            odom_msg = receive(odom_sub);
            delay = (img_seg_msg.Header.Stamp.Nsec - odom_msg.Header.Stamp.Nsec)*10^-9;
        end
        
        % Get transform: map -> camera.
        x_odom_W_VSB = [odom_msg.Pose.Pose.Position.X, ...
            odom_msg.Pose.Pose.Position.Y, odom_msg.Pose.Pose.Position.Z];
        quat_odom_W_VSB = [odom_msg.Pose.Pose.Orientation.W, ...
            odom_msg.Pose.Pose.Orientation.X, odom_msg.Pose.Pose.Orientation.Y, ...
            odom_msg.Pose.Pose.Orientation.Z];
        disp(['Taking meas. at: ', num2str(x_odom_W_VSB)]);
 
        T_W_VSB = quat2tform(quat_odom_W_VSB);
        T_W_VSB(1:3, 4) = x_odom_W_VSB';
        T_MAP_CAM = transforms.T_MAP_W * T_W_VSB * transforms.T_VSB_CAM;
        x_odom_MAP_CAM = tform2trvec(T_MAP_CAM);
        
        % Get segmented image.
        img_seg = readImage(img_seg_msg);
        
        % Update the map.
        grid_map = take_measurement_at_point(x_odom_MAP_CAM, img_seg, grid_map, ...
            map_parameters, planning_parameters);
        metrics.P_traces = [metrics.P_traces; trace(grid_map.P)];
        metrics.maps = cat(3, metrics.maps, grid_map.m);
        metrics.odoms = [metrics.odoms; odom_msg];
        
    end

    Y_sigma = sqrt(diag(grid_map.P)');
    P_post = reshape(2*Y_sigma,predict_dim_y,predict_dim_x);
    disp(['Trace after execution: ', num2str(trace(grid_map.P))]);
    disp(['Time after execution: ', num2str(get_trajectory_total_time(trajectory))]);
    gain = P_trace_prev - trace(grid_map.P);
    if (strcmp(planning_parameters.obj, 'rate'))
        cost = max(get_trajectory_total_time(trajectory), 1/planning_parameters.measurement_frequency);
        disp(['Objective after execution: ', num2str(-gain/cost)]);
    elseif (strcmp(planning_parameters.obj, 'exponential'))
        cost = get_trajectory_total_time(trajectory);
        disp(['Objective after execution: ', num2str(-gain*exp(-planning_parameters.lambda*cost))]);
    end
    
    metrics.points_meas = [metrics.points_meas; points_meas];
    metrics.times = [metrics.times; time_elapsed + times_meas'];

    % Update variables for next planning stage.
    metrics.path_travelled = [metrics.path_travelled; path_optimized];
    P_trace_prev = trace(grid_map.P);
    
    point_prev = path_optimized(end,:); % End of trajectory (not last meas. point!)
    time_elapsed = time_elapsed + get_trajectory_total_time(trajectory);  

    if (budget_spent)
        break;
    end
    
end