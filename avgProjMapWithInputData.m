function updated_map = avgProjMapWithInputData(proj_map, input_data, alpha, h, w, is_use, t)

    %==== Set variables ====
    input_points = input_data.pointcloud.Location;
    input_colors = input_data.pointcloud.Color;
    input_normals = input_data.normals;
    proj_points = proj_map.points;
    proj_colors = proj_map.colors;
    proj_normals = proj_map.normals;
    proj_ccounts = proj_map.ccounts;
    proj_times = proj_map.times;
   
    input_points=reshape(input_points,h*w,3);
    input_colors=reshape(input_colors,h*w,3);
    input_normals=reshape(input_normals,h*w,3);
    proj_points=reshape(proj_points,h*w,3);
    proj_colors=reshape(proj_colors,h*w,3);
    proj_normals=reshape(proj_normals,h*w,3);
    proj_ccounts=reshape(proj_ccounts,h*w,1);
    proj_times=reshape(proj_times,h*w,1);
    alpha=reshape(alpha,h*w,1);
    is_use=reshape(is_use,h*w,1);
    
    
   %UPDATE POINTS
     
   updated_points = proj_points;
   
   usable_input_points=input_points(is_use,:);
    
   corresponding_proj_points=proj_points(is_use,:);
   

   update_points=([proj_ccounts(is_use,:),proj_ccounts(is_use,:),proj_ccounts(is_use,:)].*corresponding_proj_points + [alpha(is_use,:),alpha(is_use,:),alpha(is_use,:)].*usable_input_points)./([proj_ccounts(is_use,:),proj_ccounts(is_use,:),proj_ccounts(is_use,:)] + [alpha(is_use,:),alpha(is_use,:),alpha(is_use,:)]);
   updated_points(is_use,:)=update_points;
   updated_points=reshape(updated_points,h,w,3);

   %UPDATE NORMALS
   
   updated_normals=proj_normals;

   usable_input_normals=input_normals(is_use,:);

   corresponding_proj_normals=proj_normals(is_use,:); 

   update_normals=([proj_ccounts(is_use,:),proj_ccounts(is_use,:),proj_ccounts(is_use,:)].*corresponding_proj_normals + [alpha(is_use,:),alpha(is_use,:),alpha(is_use,:)].*usable_input_normals)./([proj_ccounts(is_use,:),proj_ccounts(is_use,:),proj_ccounts(is_use,:)] + [alpha(is_use,:),alpha(is_use,:),alpha(is_use,:)]);
   updated_normals(is_use,:)=update_normals;
   updated_normals=reshape(updated_normals,h,w,3);

   %UPDATE COLORS
   
    updated_colors=double(proj_colors);
    usable_input_colors=double(input_colors(is_use,:));
    corresponding_proj_colors=double(proj_colors(is_use,:)); 
    update_colors=([proj_ccounts(is_use,:),proj_ccounts(is_use,:),proj_ccounts(is_use,:)].*corresponding_proj_colors + [alpha(is_use,:),alpha(is_use,:),alpha(is_use,:)].*usable_input_colors)./([proj_ccounts(is_use,:),proj_ccounts(is_use,:),proj_ccounts(is_use,:)] + [alpha(is_use,:),alpha(is_use,:),alpha(is_use,:)]);
    updated_colors(is_use,:)=update_colors;
    updated_colors=uint8(updated_colors);
    updated_colors=reshape(updated_colors,h,w,3);

   
   %UPDATE CCOUNTS
   updated_ccounts=proj_ccounts;
   usable_input_alpha=alpha(is_use,:);
   corresponding_proj_ccounts=proj_ccounts(is_use,:); 
   update_ccounts=corresponding_proj_ccounts + usable_input_alpha;
   updated_ccounts(is_use,:)=update_ccounts;
   updated_ccounts=reshape(updated_ccounts,h,w,1);
   

   %UPDATED TIMES
   
    
   update=t*ones(h*w,1);
   updated_times=update;
   updated_times=reshape(updated_times,h,w,1);
   
   
    %==== TODO: Update all the terms in the projected map using the input data ====
    %==== (Hint: apply is_use[] as a mask in vectorization) ====   
    
    % Write your code here...
    
    
    %==== Output the updated projected map in a struct ====
    updated_map = struct('points', updated_points, 'colors', updated_colors, 'normals', updated_normals, 'ccounts', updated_ccounts, 'times', updated_times);
        
end