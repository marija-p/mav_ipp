function [lattice_env] = create_lattice(map_params, planning_params)
% Create multi-dimensional lattice in the UAV configuration space.

lattice = [];

manual_lattice = 0;

if (~manual_lattice)
    
    point_coeffs = polyfit([planning_params.min_height, planning_params.max_height], ...
        [planning_params.lattice_min_height_points, 1], 1);
    
    for height = planning_params.min_height: ...
            planning_params.lattice_height_increment: ...
            planning_params.max_height
        
        num_of_points = round(point_coeffs(1)*height + point_coeffs(2));
        
        submap_edge_size = ...
            get_submap_edge_size(height, map_params, planning_params);
        half_submap_edge_size_x = (submap_edge_size.x-1)/2;
        half_submap_edge_size_y = (submap_edge_size.y-1)/2;
        % Compute distance between points on a lattice plane,
        % assuming same discritisation in x- and y-dirs.
        if (sqrt(num_of_points) == 1)
            grid_x = round(map_params.dim_x/2);
            grid_y = round(map_params.dim_y/2);
            grid_z = height;
        else
            [grid_x, grid_y] = meshgrid(linspace(half_submap_edge_size_x, ...
                map_params.dim_x-half_submap_edge_size_x, sqrt(num_of_points)), ...
                linspace(half_submap_edge_size_y, ...
                map_params.dim_y-half_submap_edge_size_y, sqrt(num_of_points)));
            grid_x = reshape(grid_x, [], 1);
            grid_y = reshape(grid_y, [], 1);
            grid_z = height*ones(size(grid_x,1),1);
        end
        
        % Add grid at current altitude level to lattice.
        lattice = [lattice; grid_x, grid_y, grid_z];
        
    end
    
    lattice_env = grid_to_env_coordinates(lattice, map_params);
    %plot3(lattice_env(:,1), lattice_env(:,2), lattice_env(:,3), '.k');
    
else
    
    lattice_env = [-14.5858, -14.4226, 1;
        -14.5858, -4.80755, 1;
        -14.5858, 4.80755, 1;
        -14.5858, 14.4226, 1;
        -4.86193, -14.4226, 1;
        -4.86193, -4.80755, 1;
        -4.86193, 4.80755, 1;
        -4.86193, 14.4226, 1;
        4.86193, -14.4226, 1;
        4.86193, -4.80755, 1;
        4.86193, 4.80755, 1;
        4.86193, 14.4226, 1;
        14.5858, -14.4226, 1;
        14.5858, -4.80755, 1;
        14.5858, 4.80755, 1;
        14.5858, 14.4226, 1;
        -11.2721, -9.80385, 9;
        -11.2721, 0, 9;
        -11.2721, 9.80385, 9;
        0, -9.80385, 9;
        0, 0, 9;
        0, 9.80385, 9;
        11.2721, -9.80385, 9;
        11.2721, 0, 9;
        11.2721, 9.80385, 9;
        -7.95837, -5.18505, 17;
        -7.95837, 5.18505, 17;
        7.95837, -5.18505, 17;
        7.95837, 5.18505, 17;
        0, 0, 25];
    
end

end
