seeds = fieldnames(logger);

for i = 1:length(seeds)
   
    figure;
    plot_metrics(logger.(seeds{i}).('nonadaptive'));
    plot_metrics(logger.(seeds{i}).('adaptive'));
    title(seeds{i})
    
end