function [xpix, ypix] = points2image(dxyz, rod, K, P, cameraParams)
%Computes the list of pixels given by the projection of P 3D points onto
%the image given the postion and intrinsics of the camera

%% Compute T matrix from dxyz and rod
T = zeros(4,4);
T(4,4) = 1; %homogeneize
T(1:3,1:3) = rod2dcm(rod);
T(1:3, 4) = dxyz';

ph = K * T * P;

xpix = ph(1,:)./ph(3,:);
ypix = ph(2,:)./ph(3,:);


% imagePoints = worldToImage(cameraParams,...
%     rod2dcm(rod),...
%     dxyz,...
%     P(1:3,:)');
% 
% xpix = imagePoints(:,1)';
% ypix = imagePoints(:,2)';