logger = [];

% Environment dimensions [m].
dim_x_env = 200;
dim_y_env = 290;


for i = 6:10
    
    % Set the seed in the CMA-ES random number generator.
    [matlab_params, planning_params, opt_params, map_params] = ...
        load_params(dim_x_env, dim_y_env);
    opt_params.seed = i;
    
    % Non-adaptive planning.
    planning_params.do_adaptive_planning = 0;
    metrics = ipp_node_fun(matlab_params, planning_params, ...
        opt_params, map_params);
    logger.(['seed', num2str(i)]).('nonadaptive') = metrics;
    
    % Adaptive planning.
    planning_params.do_adaptive_planning = 1;
    metrics = ipp_node_fun(matlab_params, planning_params, ...
        opt_params, map_params);
    logger.(['seed', num2str(i)]).('adaptive') = metrics;
    
    disp(['Evaluated Seed ', num2str(i)]);
    
end