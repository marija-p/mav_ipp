function grid_map = ...
    create_random_map(dim_x, dim_y, num_classes)

    grid_map = zeros(dim_x, dim_y, num_classes);
    
    for i = 1:dim_x
        for j = 1:dim_y
            grid_map(i,j,:) = randfixedsum(3,1,1,0,1);
        end
    end
    
    grid_map = prob_to_logodds(grid_map);

end

