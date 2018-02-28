
%     
%     % Write your code here...
%     K=[fx 0 cx;0 fy cy; 0 0 1];
%     t_newform=invert(tform);
%     new_points = pctransform(fusion_map.pointcloud,t_newform);
%     n_points = (K*new_points.Location');
%    
% %   Normalize
%     u = n_points(1,:)./n_points(3,:);
%     v = n_points(2,:)./n_points(3,:);
%     pixels=[u;v]';
%     pixels=round(pixels);
%     
%     inside = pixels(:,1) <= w;
%     inside = and(inside, pixels(:,2) <= h);
%     inside = and(inside, pixels(:,1) > 0);
%     inside = and(inside, pixels(:,2) > 0);
%     valid_indices=find(inside);
%     new_pixels=pixels(valid_indices,:);
%     
%     [Latest_pixels,index,~] = unique(new_pixels,'rows');
%     valid_indices = valid_indices(index);
%     
%     proj_map= zeros(w, h, 3);
% %     proj_map(new_pixels(:,1),new_pixels(:,2),1)=n_points(valid_indices,1);
%     linearInd = sub2ind(size(proj_map), new_pixels(:,1), new_pixels(:,2));
function [proj_map, proj_flag] = projMapToFrame(fusion_map, h, w, tform, cam_param)
    
    %==== Set parameters ====
    fx = cam_param(1);
    fy = cam_param(2);
    cx = cam_param(3);
    cy = cam_param(4);
% TO GET PROJ POINTS[]
    K= [fx 0 cx;
        0 fy cy;
        0  0  1];
        
    itform=invert(tform);
    fusion_map_cameraframe=pctransform(fusion_map.pointcloud,itform);
    valid_indices=find(abs(fusion_map_cameraframe.Location(:,1))<=abs(cx*fusion_map_cameraframe.Location(:,3)/fx) & abs(fusion_map_cameraframe.Location(:,2))<=abs(cy*fusion_map_cameraframe.Location(:,3)/fy));
    fusion_map_camera_valid=fusion_map_cameraframe.Location(valid_indices,:); %camera frame valid points
    
    pixel_coordinate=(K*fusion_map_camera_valid')';
    pixel_coordinate(:,1)=pixel_coordinate(:,1)./pixel_coordinate(:,3);
    pixel_coordinate(:,2)=pixel_coordinate(:,2)./pixel_coordinate(:,3);
    pixel_coordinate(:,3)=pixel_coordinate(:,3)./pixel_coordinate(:,3);
    pixel_coordinate=round(pixel_coordinate) + 1;
    pixel_coordinate(:,3)=pixel_coordinate(:,3)-1;
    [unique_pixel_coordinate,ia,~]=unique(pixel_coordinate,'rows');
    unique_indices=valid_indices(ia,1);
    
    proj_points_2d=zeros(h*w,3);
    index_2d=sub2ind([h,w],unique_pixel_coordinate(:,2),unique_pixel_coordinate(:,1));
    proj_flag_2=index_2d;
    proj_points_2d(index_2d,:)=fusion_map.pointcloud.Location(unique_indices,:);  %%%the step where multiple get projected
    proj_points=reshape(proj_points_2d,480,640,3);
    proj_colors_2d=zeros(h*w,3);
    proj_colors_2d(index_2d,:)=fusion_map.pointcloud.Color(unique_indices,:);
    proj_colors=reshape(proj_colors_2d,480,640,3);
    proj_colors=uint8(proj_colors);
   proj_normals_2d=zeros(h*w,3); 
   proj_normals_2d(index_2d,:)=fusion_map.normals(unique_indices,:);
   proj_normals=reshape(proj_normals_2d,480,640,3);
   proj_ccounts_2d=zeros(h*w,1); 
   proj_ccounts_2d(index_2d,:)=fusion_map.ccounts(unique_indices,:);
   proj_ccounts=reshape(proj_ccounts_2d,480,640);
   
   proj_times_2d=zeros(h*w,1); 
   proj_times_2d(index_2d,:)=fusion_map.times(unique_indices,:);
   proj_times=reshape(proj_times_2d,480,640);
     proj_flag_1=unique_indices;
    proj_map = struct('points', proj_points, 'colors', proj_colors, 'normals', proj_normals, 'ccounts', proj_ccounts, 'times', proj_times);
    proj_flag=struct('fusionmapindex',proj_flag_1,'pixelmapindex',proj_flag_2);  
end