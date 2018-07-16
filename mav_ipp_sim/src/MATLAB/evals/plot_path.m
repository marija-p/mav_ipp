function [] = plot_path(path, planning_params)
% Visualizes a trajectory from a list of control points.

t = [];
p = [];
t_meas = [];
p_meas = [];

% Many polynomials.
if isfield(planning_params, 'control_points')
    
    % Loop through all polynomial trajectories that were executed,
    % and stack times/measurements for plotting.
    for i = 1:planning_params.control_points:size(path,1)

        % Create the (semi-global) trajectory.
        trajectory = ...
            plan_path_waypoints(path(i:i+planning_params.control_points-1,:), ...
            planning_params.max_vel, ...
            planning_params.max_acc);
        [t_poly, p_poly] = sample_trajectory(trajectory, 0.1);
        [t_meas_poly, p_meas_poly] = sample_trajectory(trajectory, ...
            1/planning_params.measurement_frequency);
        if (i == 1)
            t = [t; t_poly'];
            t_meas = [t_meas; t_meas_poly'];
        else
            t = [t; t(end) + t_poly'];
            t_meas = [t_meas; t_meas(end) + t_meas_poly'];
        end
        p = [p; p_poly];
        p_meas = [p_meas; p_meas_poly];
        
        %disp('Polynomial meas. points: ')
        %disp(p_meas_poly)
        
    end
    
% Only one polynomial.
else
    
    % Create the (global) trajectory.
    trajectory = plan_path_waypoints(path, planning_params.max_vel, ...
        planning_params.max_acc);
    [t, p] = sample_trajectory(trajectory, 0.1);
    [t_meas, p_meas] = sample_trajectory(trajectory, ...
        1/planning_params.measurement_frequency);
    
end

hold on
% Visualize trajectory.
ind = find(t == 100);
cline(p(1:ind,1), p(1:ind,2), p(1:ind,3), t(1:ind));
% Visualize control points.
%scatter3(path(:,1), path(:,2), path(:,3), 140, 'xk');
% Visualize measurements.
colors_meas = linspace(0, t_meas(end), size(p_meas,1));

% Silly bug with 3 points.
% https://ch.mathworks.com/matlabcentral/newsreader/view_thread/136731
if isequal(size(colors_meas),[1 3])
    colors_meas = colors_meas';
end

ind = find(t_meas == 100);
scatter3(p_meas(1:ind,1), p_meas(1:ind,2), p_meas(1:ind,3), 30, ...
    colors_meas(1:ind), 'filled');

xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
axis([-110 110 -160 160 0 200])
colormap jet
grid on
c = colorbar;
ylabel(c, 'Time (s)')
view(3)
%legend('Path', 'Control pts.', 'Meas. pts.', 'Location', 'northeast')
%legend('Path', 'Meas. pts.', 'Location', 'northeast')
daspect([1 1 1])
set(gca, 'FontName', 'Times')
set(gca, 'FontSize', 11)
set(gcf, 'Color', 'w')
set(gcf, 'Position', [1, 1, 309, 399])
%disp(p_meas)

disp(ind)

end