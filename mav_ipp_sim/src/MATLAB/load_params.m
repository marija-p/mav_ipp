function [matlab_params, planning_params, ...
    opt_params, map_params] = load_params(dim_x_env, dim_y_env)
% Loads defaults IPP parameters for RotorS-based simulation.

% Random number generator
matlab_params.seed_num = 2;
matlab_params.visualize = 0;

% Height-dependent sensor (classifier) model - causal
planning_params.classifier_altitudes = [50, 70, 100];
% Indexing: (class 1, class 2, class 3). Rows: predict, cols: true.
planning_params.classifier_conf_matrices(:,:,1) = ...
    [0.9614, 0.0219, 0.0166;
    0.2749, 0.7007, 0.0244;
    0.2769, 0.0430, 0.6801];
planning_params.classifier_conf_matrices(:,:,2) = ...
    [0.9815, 0.0138, 0.0047;
    0.3183, 0.6705, 0.0112;
    0.3778, 0.0483, 0.5739];
planning_params.classifier_conf_matrices(:,:,3) = ...
    [0.9766, 0.0224, 0.0010;
    0.5166, 0.4820, 0.0013;
    0.5232, 0.0865, 0.3903];

% Camera fields of view (FoV)
planning_params.sensor_fov_angle_x = 35.4;
planning_params.sensor_fov_angle_y = 47.2;
planning_params.min_height = 40;
planning_params.max_height = 160;
planning_params.max_vel = 15;        % [m/s]
planning_params.max_acc = 20;        % [m/s^2]
planning_params.time_budget = 5000;  % [s]

% Frequency at which to take measurements along a path [Hz]
planning_params.measurement_frequency = 0.1;

% Number of control points for a polynomial (start point fixed)
planning_params.control_points = 5;

% Number of lattice points at lowest altitude level
planning_params.lattice_min_height_points = 16;
% Distance between successive altitude levels on the lattice
planning_params.lattice_height_increment = 40;

% Minimum distance before a waypoint is considered reached.
planning_params.achievement_dist = 2;

% TODO: active planning
% Whether to use the threshold value for active planning.
planning_params.use_threshold = 1;

opt_params.max_iters = 200;
opt_params.opt_method = 'cmaes'; % 'fmc'/cmaes'/'none'/'bo'
% Covariances in each search dimension
opt_params.cov_x = 20;
opt_params.cov_y = 20;
opt_params.cov_z = 10;

% Map resolution [m/cell]
map_params.resolution = 5;
% Map dimensions [cells]
map_params.dim_x = dim_x_env/map_params.resolution;
map_params.dim_y = dim_y_env/map_params.resolution;
% Position of map in the environment [m]
map_params.position_x = -dim_x_env / 2;
map_params.position_y = -dim_y_env / 2;

end

