planning_params.do_adaptive_planning = 0;
planning_params.interesting_class_ind = 2;
planning_params.lower_threshold = prob_to_logodds(0.4);

tic;
 
for i = 1:1000
    if (planning_params.do_adaptive_planning)
        grid_map_interesting = ...
            grid_map(2:end-1, 2:end-1, planning_params.interesting_class_ind);
        entropy = ...
            get_map_entropy(grid_map_interesting(find(grid_map_interesting >= planning_params.lower_threshold)));
    else
        entropy = get_map_entropy(grid_map(2:end-1, 2:end-1,:));
    end
end
 
time_elapsed = toc;
disp(toc);
disp(entropy)