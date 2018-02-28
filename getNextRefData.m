function next_ref_data = getNextRefData(updated_map, ds_ratio)

    %==== TODO: Generate the downsampled pointcloud and normals ====
    %==== (Hint: Refer to the "Downsample input data" part in "ICP_FUSION.m") ====
    
    % Write your code here...
    to_ds_pointcloud.Location=updated_map.points;
    to_ds_pointcloud.Color=updated_map.colors;
    
    [next_ref_pointcloud, next_ref_normals] = downsampleData(to_ds_pointcloud, updated_map.normals, ds_ratio);
    next_ref_data = struct('pointcloud', next_ref_pointcloud, 'normals', next_ref_normals);
end
    