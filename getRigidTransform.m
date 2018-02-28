function [tform,valid_pair_num,error] = getRigidTransform(new_pointcloud, ref_pointcloud, ref_normals)

%==== Initialize parameters ====
iter_num = 6;
d_th = 0.05;
m = size(new_pointcloud.Location, 1);
n = size(new_pointcloud.Location, 2);
tform = affine3d(eye(4));
G=zeros(3,6);
N=zeros(m*n,3);
%==== Main iteration loop ====
for iter = 1:iter_num
    
    %==== Set variables ====
    new_pts = new_pointcloud.Location;
    ref_pts = ref_pointcloud.Location;
    
    %==== For each reference point, find the closest new point within a local patch of size 3-by-3 ====
    %==== (Notice: assoc_pts[] has the same size and format as new_pts[] and ref_pts[]) ====
    %==== (Notice: assoc_pts[i, j, :] = [0 0 0] iff no point in new_pts[] matches to ref_pts[i, j, :]) ====
    assoc_pts = findLocalClosest(new_pts, ref_pts, m, n, d_th);
    
    %==== Set the sizes of matrix A[] and vertor b[] of normal equation: A'*A*x = A'*b ====
    A = zeros(m*n, 6);
    b = zeros(m*n, 1);
    
    %==== declare the number of point pairs that are used in this iteration ====
    valid_pair_num = 0;
    
    %==== TODO: Assign values to A[] and b[] ====
    %==== (Notice: the format of the desired 6-vector is: xi = [r_x r_y r_z t_x t_y t_z]') ====
    
    % Write your code here...
    
    %FOR A MATRIX
%     G=zeros(3,6);
%     N=zeros(m*n,3);
    k=0;
    for i=1:m
        for j=1:n
            
            G=[toSkewSym(assoc_pts(i,j,:)) eye(3)];
            N(k+j,:)=ref_normals(i,j,1);
            N11=N(k+j,1);
            N(k+j,:)=ref_normals(i,j,2);
            N12=N(k+j,2);
            N(k+j,:)=ref_normals(i,j,3);
            N13=N(k+j,3);
            A(k+j,:)=[N11 N12 N13]*G;
            b(k+j,1)=[N11 N12 N13]*[ref_pts(i,j,1)-assoc_pts(i,j,1);ref_pts(i,j,2)-assoc_pts(i,j,2);  ref_pts(i,j,3)-assoc_pts(i,j,3)];
            
            if assoc_pts(i, j, :) ~=0;
                valid_pair_num=valid_pair_num+1;
            else
                A(k+j,:)=0;
                b(k+j,1)=0;
            end
            
        end
        k=k+40;
    end
    
    
    
    %==== TODO: Solve for the 6-vector xi[] of rigid body transformation from A[] and b[] ====
    
    % Write your code here...
    xi=A\b;
    
    %==== Coerce xi[] back into SE(3) ====
    %==== (Notice: tmp_tform[] is defined in the format of right-multiplication) ====
    R = toSkewSym(xi(1:3)) + eye(3);
    [U,~,V] = svd(R);
    R = U*V';
    T = [R [0 ; 0 ; 0] ; [xi(4:6)' 1]];
    tmp_tform = affine3d(T);
    
    %==== TODO: Update the transformation and the pointcloud for the next iteration ====
    %==== (Hint: use affine3d() and pctransform() functions) ====
    %==== (Hint: be careful of the format of tform[] and the affine3d() function) ====
    
    % Write your code here...
    c=tform.T*tmp_tform.T;
    tform = affine3d(c);
    new_pointcloud= pctransform(new_pointcloud,tmp_tform);
end

%==== Find RMS error of point-plane registration ====
error = sqrt(sum((A*xi - b).^2)/valid_pair_num);
end
