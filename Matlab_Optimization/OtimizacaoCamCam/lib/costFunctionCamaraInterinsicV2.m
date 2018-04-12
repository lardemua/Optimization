function [cost] = costFunctionCamaraInterinsicV2(x, worldPoints, s, N, K, vplot, cameraParams)

persistent num_calls;
if isempty(num_calls)
    num_calls = 0;
end
num_calls = num_calls + 1;

%% Cost Function

% Geometric transformation
for k=1:K
    
    dxyz = x( (k-1)*N+1 : (k-1)*N+3 );
    rod = x( (k-1)*N+4 : (k-1)*N+6 );
    intrinsic_vector = x( (k-1)*N+7 : (k-1)*N+10 );
    intrinsics = vectorToInterinsic( intrinsic_vector );
    [s{k}.xpix, s{k}.ypix] = points2imageTeste(dxyz, rod, intrinsics, worldPoints, cameraParams);
    
    %Draw iterate projections
    subplot(K,K+1,k*(K+1))
    set(s{k}.handle, 'XData',s{k}.xpix,'YData', s{k}.ypix );
    set(s{k}.texthandle,'Position', [s{k}.xpix(1) s{k}.ypix(1)])
    
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
subplot(K,K+1,vplot);
iter_str = num2str(num_calls);
cost_str = num2str(cost);

titleStr = ['Camera positions ' '|| ' 'Func-count: ' iter_str '|| ' 'Cost: ' cost_str];

title(titleStr)
