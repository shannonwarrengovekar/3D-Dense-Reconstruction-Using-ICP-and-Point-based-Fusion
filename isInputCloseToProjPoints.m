function is_close = isInputCloseToProjPoints(input_points, proj_points, dist2_th)

    %==== TODO: Output a boolean matrix which represents if each corresponding point pairs are close enough given dist2_th ====
    % Write your code here...
%     temp1=input_points-proj_points;
        is_close=logical(dist2_th >((input_points(:,:,1)-proj_points(:,:,1)).^2+(input_points(:,:,2)-proj_points(:,:,2)).^2+(input_points(:,:,3)-proj_points(:,:,3)).^2));
%         is_close=logical(sum(temp1.^2)=<dist2_th)
end
    