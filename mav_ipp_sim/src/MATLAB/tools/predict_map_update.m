function grid_map = ...
    predict_map_update(pos, grid_map, map_params, planning_params)
% Predicts grid map update at an unvisited UAV position using
% height-dependent sensor model (camera + classifier).
% --
% Inputs:
% pos: current [x,y,z] UAV position [m] - env. coordinates
% grid_map: current occupancy grid map
% ---
% Outputs:
% grid map
% ---
% M Popovic 2018
%

%% Measurement Prediction
% Compute current FoV (camera footprint).
submap_edge_size = ...
    get_submap_edge_size(pos(3), map_params, planning_params);
submap_coordinates = ...
    get_submap_coordinates(pos, submap_edge_size, map_params);
submap = grid_map(submap_coordinates.yd:submap_coordinates.yu, ...
    submap_coordinates.xl:submap_coordinates.xr, :);
submap = logodds_to_prob(submap);
submap = round(logodds_to_prob(submap));

%% Sensor Model
% Find the confusion matrix corresponding to the current altitude.
[~, ind_altitude] = min(abs(planning_params.classifier_altitudes - pos(3)));
conf_matrix = planning_params.classifier_conf_matrices(:,:,ind_altitude);

% Mapping strategy #2 (3 layers).
% Initialize likelihood matrix for each cell in the submap.
likelihoods = zeros(size(submap));

% Calculate observation likelihoods based on possible observations.
for i = 1:size(likelihoods,1)
    for j = 1:size(likelihoods,2)
        for k = 1:size(likelihoods,3)
            if (submap(i,j,k) == 1)
                likelihoods(i,j,k) = conf_matrix(k,k);
            else
                likelihoods(i,j,k) = sum(conf_matrix(k,setdiff(1:3, k)));
            end
        end
    end
end
end

% Calculate observation likelihoods based on expectation.
% % LAYER 1 [Class 1].
% likelihoods(:,:,1) = submap(:,:,1)*conf_matrix(1,1) + ...
%     submap(:,:,1)*(conf_matrix(1,2) + conf_matrix(1,3));
% % LAYER 2 [Class 2].
% likelihoods(:,:,2) = submap(:,:,2)*conf_matrix(2,2) + ...
%     submap(:,:,2)*(conf_matrix(2,1) + conf_matrix(2,3));
% % LAYER 2 [Class 3].
% likelihoods(:,:,3) = submap(:,:,3)*conf_matrix(3,3) + ...
%     submap(:,:,3)*(conf_matrix(3,1) + conf_matrix(3,2));

% Mapping strategy #1 (3 classes).
% Find the most likely class for each cell in the submap.
% [~, ind_classes] = max(submap, [], 3);

% % Determine likelihoods for each cell in the submap.
% likelihoods = zeros(size(submap));

% for i = 1:size(likelihoods,1)
%     for j = 1:size(likelihoods,2)
%         for k = 1:size(likelihoods,3)
%             likelihoods(i,j,k) = conf_matrix(k, ind_classes(i,j));
%             %disp(['Map value (TRUE): ', classes{k}, '. Meas. (PREDICT): ', classes{ind_classes(i,j)}]);
%             %disp(['Likelihood: ', num2str(conf_matrix(k,ind_classes(i,j)))]);
%         end
%     end
% end
% 

likelihoods = prob_to_logodds(likelihoods);
 
% Update the grid map based on prediction.
grid_map(submap_coordinates.yd:submap_coordinates.yu, ...
    submap_coordinates.xl:submap_coordinates.xr, :) = ...
    grid_map(submap_coordinates.yd:submap_coordinates.yu, ...
    submap_coordinates.xl:submap_coordinates.xr, :) + likelihoods;

end
