function [matlab_params, planning_params, ...
    opt_params, map_params] = load_params(dim_x_env, dim_y_env)
% Loads defaults IPP parameters for RotorS-based orthomosaic simulation.

% Random number generator
matlab_params.seed_num = 2;
matlab_params.visualize = 0;

% Height-dependent sensor (classifier) model
planning_params.classifier_altitudes = [5, 10, 15];
% Indexing: (class 1, class 2, class 3). Rows: predict, cols: true.
planning_params.classifier_conf_matrices(:,:,1) = ...
    [0.95, 0.07, 0.31;
    0.03, 0.80, 0.04;
    0.02, 0.13, 0.65];
planning_params.classifier_conf_matrices(:,:,2) = ...
    [0.89, 0.11, 0.28;
    0.06, 0.75, 0.13;
    0.05, 0.14, 0.59];
planning_params.classifier_conf_matrices(:,:,3) = ...
    [0.80, 0.17, 0.29;
    0.11, 0.63, 0.21;
    0.09, 0.20, 0.50];

% Camera fields of view (FoV)
planning_params.sensor_fov_angle_x = 35.4;
planning_params.sensor_fov_angle_y = 47.2;
planning_params.min_height = 5;
planning_params.max_height = 15;
planning_params.max_vel = 1.5;        % [m/s]
planning_params.max_acc = 2;          % [m/s^2]
planning_params.time_budget = 5000;   % [s]

% Frequency at which to take measurements along a path [Hz]
planning_params.measurement_frequency = 0.2;

% Number of control points for a polynomial (start point fixed)
planning_params.control_points = 4;

% Number of lattice points at lowest altitude level
planning_params.lattice_min_height_points = 16;
% Distance between successive altitude levels on the lattice
planning_params.lattice_height_increment = 5;

% TODO: active planning
% Whether to use the threshold value for active planning
planning_params.use_threshold = 1;

opt_params.max_iters = 15;
opt_params.opt_method = 'cmaes'; % 'fmc'/cmaes'/'none'/'bo'
% Covariances in each search dimension
opt_params.cov_x = 0.2;
opt_params.cov_y = 0.2;
opt_params.cov_z = 0.1;

% Map resolution [m/cell]
map_params.resolution = 0.1;
% Map dimensions [cells]
map_params.dim_x = dim_x_env/map_params.resolution;
map_params.dim_y = dim_y_env/map_params.resolution;
% Position of map in the environment [m]
map_params.position_x = -dim_x_env / 2;
map_params.position_y = -dim_y_env / 2;

end

