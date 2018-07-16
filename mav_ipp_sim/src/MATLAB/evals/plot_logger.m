close all;

methods = fieldnames(logger);

do_plot = 1;
show_legend = 1;

rescale_factor = 0.89;
text_size = 10.5;
line_width = 1;
plot_aspect_ratio = [1 2 1];

time_vector = 0:0.1:401;


entropies = zeros(length(methods),length(time_vector));
rmses = zeros(length(methods),length(time_vector));
entropies_interesting = zeros(length(methods),length(time_vector));
rmses_interesting = zeros(length(methods),length(time_vector));

for i = 1:length(methods)
    
    times = logger.(methods{i}).times;
    entropy = logger.(methods{i}).entropies;
    rmse = logger.(methods{i}).rmses;
    entropy_interesting = logger.(methods{i}).entropies_interesting;
    rmse_interesting = logger.(methods{i}).rmses_interesting;
    
    ts = timeseries(entropy, times);
    ts_resampled = resample(ts, time_vector, 'zoh');
    entropies(i,:) = ts_resampled.data';
    ts = timeseries(rmse, times);
    ts_resampled = resample(ts, time_vector, 'zoh');
    rmses(i,:) = ts_resampled.data';
    ts = timeseries(entropy_interesting, times);
    ts_resampled = resample(ts, time_vector, 'zoh');
    entropies_interesting(i,:) = ts_resampled.data';
    ts = timeseries(rmse_interesting, times);
    ts_resampled = resample(ts, time_vector, 'zoh');
    rmses_interesting(i,:) = ts_resampled.data';
    
end

colours = [0.8500    0.3250    0.0980;
    0    0.4470    0.7410;
  %  0.9290    0.6940    0.1250;
 %   0.4940    0.1840    0.5560;
    0.4660    0.6740    0.1880;
0.6350 0.0780 0.1840];

if (do_plot)
    
    % Entropy
    subplot(1,3,1)
    hold on
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'on'      , ...
        'YMinorTick'  , 'off'      , ...
        'YTickMode'   , 'auto'  , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YScale'      , 'log'     , ...
        'YGrid'       , 'on'      , ...
        'LineWidth'   , 1         , ...
        'FontSize'    , text_size , ...
        'FontName'    , 'Times' , ...
        'LooseInset'  , max(get(gca,'TightInset'), 0.02));
    
    % Convert to bits.
    % https://www.physicsforums.com/threads/converting-between-bits-nats-and-dits.521115/
    for i = 1:length(methods)
        plot(time_vector, entropies(i,:)*log2(exp(1)), ...
            'Color', colours(i,:), 'LineWidth', line_width)
    end
    axis([0 time_vector(end) 0 10000])
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('Entropy (bits)');
    set([h_xlabel, h_ylabel], 'FontName'   , 'Times');
    rescale_axes(rescale_factor)
    pbaspect(gca, plot_aspect_ratio)
    hold off
    
    % Entropy - interesting areas
    subplot(1,3,2)
    hold on
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'on'      , ...
        'YMinorTick'  , 'off'      , ...
        'YTickMode'   , 'auto'  , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YScale'      , 'log'     , ...
        'YGrid'       , 'on'      , ...
        'LineWidth'   , 1         , ...
        'FontSize'    , text_size , ...
        'FontName'    , 'Times' , ...
        'LooseInset'  , max(get(gca,'TightInset'), 0.02));
    
    % Convert to bits.
    for i = 1:length(methods)
        plot(time_vector, entropies_interesting(i,:)*log2(exp(1)), ...
            'Color', colours(i,:), 'LineWidth', line_width)
    end
    axis([0 time_vector(end) 0 1000])
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('Entropy - interesting (bits)');
    set([h_xlabel, h_ylabel], 'FontName'   , 'Times');
    rescale_axes(rescale_factor)
    pbaspect(gca, plot_aspect_ratio)
    hold off
    
    % RMSE - interesting areas
    subplot(1,3,3)
    hold on
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'on'      , ...
        'YMinorTick'  , 'off'      , ...
        'YTickMode'   , 'auto'  , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YScale'      , 'linear'     , ...
        'YGrid'       , 'on'      , ...
        'LineWidth'   , 1         , ...
        'FontSize'    , text_size , ...
        'FontName'    , 'Times' , ...
        'LooseInset'  , max(get(gca,'TightInset'), 0.02));
    
    for i = 1:length(methods)
        plot(time_vector, rmses_interesting(i,:), ...
            'Color', colours(i,:), 'LineWidth', line_width)
    end
    axis([0 time_vector(end) 0.2 0.6])
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('RMSE - interesting');
    set([h_xlabel, h_ylabel], 'FontName'   , 'Times');
    rescale_axes(rescale_factor)
    pbaspect(gca, plot_aspect_ratio)
    yticks([0.2:0.1:0.6])
    hold off
    
end

if (show_legend)
    h_legend = legend('Adaptive', 'Non-adaptive', 'Cov. 1', 'Cov. 2');
    set(h_legend, 'Location', 'SouthOutside');
    set(h_legend, 'orientation', 'horizontal')
    set(h_legend, 'box', 'off');
    set(h_legend, 'FontName', 'Times');
end

set(gcf, 'Color', 'w')
set(gcf, 'Position', [675   422   703   531])