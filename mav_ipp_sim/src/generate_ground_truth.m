% Orthomosaic dimensions.
[ortho_dim_y, ortho_dim_x] = size(val_labels_cropped);

% Map dimensions.
dim_x = 40;
dim_y = 58;

ground_truth = zeros(dim_y, dim_x, 3);

% Extract ground truth by checking classes for each block of pixels (cell).
pixel_indices_y = round(linspace(1, ortho_dim_y, dim_y+1));
pixel_indices_x = round(linspace(1, ortho_dim_x, dim_x+1));

% Class labels.
label_1 = [17];
label_2 = [1,3,4];
label_3 = setdiff(0:18, [label_1, label_2]);

for i = 1:numel(pixel_indices_x)-1
    for j = 1:numel(pixel_indices_y)-1
        
       % Extract block for this cell.
       pixel_data = ...
           val_labels_cropped(pixel_indices_y(j):pixel_indices_y(j+1), ...
           pixel_indices_x(i):pixel_indices_x(i+1));
       pixel_data = reshape(pixel_data,1,[]);
       ground_truth(j,i,1) =  any(ismember(pixel_data, label_1));
       ground_truth(j,i,2) =  any(ismember(pixel_data, label_2));
       ground_truth(j,i,3) =  any(ismember(pixel_data, label_3));
       
    end
end

% Debugging - visualisation.
classes = {'Water', 'BVR', 'Background'};
figure;
set(gcf, 'Position', [675, 608, 1006, 370]);
for k = 1:3
    subplot(1,3,k)
    imagesc(ground_truth(:,:,k))
    caxis([0 1])
    %set(gca,'YDir','normal')
    title(classes{k})
    axis equal
end