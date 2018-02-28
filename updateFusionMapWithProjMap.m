function fusion_map = updateFusionMapWithProjMap(fusion_map, updated_map, h, w, proj_flag)

    %==== TODO: Merge the updated map with the remaining part of the old fusion map ====
    
    % Write your code here...
    updated_map_points_2d=reshape(updated_map.points,h*w,3);
    updated_map_colors_2d=reshape(updated_map.colors,h*w,3);
    updated_map_normals_2d=reshape(updated_map.normals,h*w,3);
    updated_map_ccounts_2d=reshape(updated_map.ccounts,h*w,1);
    updated_map_times_2d=reshape(updated_map.times,h*w,1);
 
    
    map_points=fusion_map.pointcloud.Location;
    map_colors=fusion_map.pointcloud.Color;
    map_normals=fusion_map.normals;
    map_ccounts=fusion_map.ccounts;
    map_times=fusion_map.times;
    
    map_points(proj_flag.fusionmapindex,:)=updated_map_points_2d(proj_flag.pixelmapindex,:);
    map_colors(proj_flag.fusionmapindex,:)=updated_map_colors_2d(proj_flag.pixelmapindex,:);
    map_normals(proj_flag.fusionmapindex,:)= updated_map_normals_2d(proj_flag.pixelmapindex,:);
    map_ccounts(proj_flag.fusionmapindex,:)=updated_map_ccounts_2d(proj_flag.pixelmapindex,:);
    map_times(proj_flag.fusionmapindex,:)=updated_map_times_2d(proj_flag.pixelmapindex,:);
    
    updated_map_points_2d(proj_flag.pixelmapindex,:) = [];
    updated_map_colors_2d(proj_flag.pixelmapindex,:) = [];
    updated_map_normals_2d(proj_flag.pixelmapindex,:) = [];
    updated_map_ccounts_2d(proj_flag.pixelmapindex,:) = [];
    updated_map_times_2d(proj_flag.pixelmapindex,:) = [];
    
    map_points = [map_points; updated_map_points_2d];
    map_colors = [map_colors; updated_map_colors_2d];
    map_normals = [map_normals; updated_map_normals_2d];
    map_ccounts = [map_ccounts; updated_map_ccounts_2d];
    map_times = [map_times; updated_map_times_2d];
    
    %==== Output the final point-based fusion map in a struct ====
    map_pointcloud = pointCloud(map_points, 'Color', map_colors);
    fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals, 'ccounts', map_ccounts, 'times', map_times);
      
end 