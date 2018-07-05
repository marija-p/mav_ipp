function [] = plot_metrics(metrics)
% Plots logged informative metrics.

do_plot = 1;

text_size = 10.5;

time_vector = 0:0.1:metrics.times(end);

times = metrics.times;
entropies = metrics.entropies;
rmses = metrics.rmses;
entropies_interesting = metrics.entropies_interesting;

ts = timeseries(entropies, times);
ts_resampled = resample(ts, time_vector, 'zoh');
entropies_resampled = ts_resampled.data';
ts = timeseries(rmses, times);
ts_resampled = resample(ts, time_vector, 'zoh');
rmses_resampled = ts_resampled.data';
ts = timeseries(entropies_interesting, times);
ts_resampled = resample(ts, time_vector, 'zoh');
entropies_interesting_resampled = ts_resampled.data';

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
        'LooseInset'  , max(get(gca,'TightInset'), 0.02));
    
    % Convert to bits.
    % https://www.physicsforums.com/threads/converting-between-bits-nats-and-dits.521115/
    plot(time_vector, entropies_resampled*log2(exp(1)))
    axis([0 time_vector(end) 0 7000])
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('Entropy (bits)');
    set([h_xlabel, h_ylabel], 'FontName'   , 'Helvetica');
    hold off

    % RMSE
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
        'YScale'      , 'linear'     , ...
        'YGrid'       , 'on'      , ...
        'LineWidth'   , 1         , ...
        'FontSize'    , text_size , ...
        'LooseInset'  , max(get(gca,'TightInset'), 0.02));
    
    plot(time_vector, rmses_resampled)
    axis([0 time_vector(end) 0.1 0.4])
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('RMSE');
    set([h_xlabel, h_ylabel], 'FontName'   , 'Helvetica');
    hold off
    
    % Entropy - interesting areas
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
        'YScale'      , 'log'     , ...
        'YGrid'       , 'on'      , ...
        'LineWidth'   , 1         , ...
        'FontSize'    , text_size , ...
        'LooseInset'  , max(get(gca,'TightInset'), 0.02));
    
    % Convert to bits.
    % https://www.physicsforums.com/threads/converting-between-bits-nats-and-dits.521115/
    plot(time_vector, entropies_interesting_resampled*log2(exp(1)))
    axis([0 time_vector(end) 0 7000])
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('Entropy - interesting areas (bits)');
    set([h_xlabel, h_ylabel], 'FontName'   , 'Helvetica');
    hold off
    
end

set(gcf, 'Position', [675   671   873   307])

end
