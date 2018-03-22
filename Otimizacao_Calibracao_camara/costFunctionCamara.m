function [cost] = costFunctionCamara(x, intrinsics, worldPoints, s, N, K, vplot)
%% Cost Function
% Geometric transformation
for k=1:K
    
    dxyz = x( (k-1)*N+1 : (k-1)*N+3 );
    rod = x( (k-1)*N+4 : (k-1)*N+6 );
    [s{k}.xpix, s{k}.ypix] = points2image(dxyz, rod, intrinsics, worldPoints);
    
    %Draw iterate projections
%     subplot(2,K,k+K);
    subplot(K,K+1,k*(K+1))
    set(s{k}.handle, 'XData',s{k}.xpix,'YData', s{k}.ypix );
    set(s{k}.texthandle,'Position', [s{k}.xpix(1) s{k}.ypix(1)])
    
%     subplot(2,K,[1 K])
    subplot(K,K+1,vplot);
    s{k}.cam.Orientation = rod2dcm(rod)';
    s{k}.cam.Location = -dxyz * s{k}.cam.Orientation;
    set(s{k}.camLine, 'XData',[worldPoints(1,1) s{k}.cam.Location(1)],...
    'YData',[worldPoints(2,1) s{k}.cam.Location(2)],...
    'ZData',[worldPoints(3,1) s{k}.cam.Location(3)]);
    pause(10^-28)
    
end

% Cost calculation
sum_dist = 0;

for ni = 1:size(s,2)
    sum_dist =  sum_dist + ...
        sum((s{ni}.image_points(:,1) - s{ni}.xpix(:)).^2 + ...
        (s{ni}.image_points(:,2) - s{ni}.ypix(:)).^2);
end

fc = sqrt(sum_dist);
cost = fc;