function [xpix, ypix] = points2imageTeste(dxyz, rod, K, P, cameraParams)
%Computes the list of pixels given by the projection of P 3D points onto
%the image given the postion and intrinsics of the camera

%% Compute T matrix from dxyz and rod
T = zeros(4,4);
T(4,4) = 1; %homogeneize
% T1 = T;
% T2 = T;


T(1:3,1:3) = rod2dcm(rod);
T(1:3, 4) = dxyz';


% T1(1:3,1:3) = rod2dcm(rod)
% 
% 
% T2(1:3, 1:3) = eye(3,3);
% T2(1:3, 4) = dxyz'
% 
% T3 = T2 * T1
% 
% T3_outro = T1 * T2
% 
% T3_inv = inv(T3)

% Calculation of the point in the image in relation to the chess reference
% points_in_camera_coord = T * P


ph = K * T * P;
% ph = K  * T * [0.5 0 2 1]';

xpix = ph(1,:)./ph(3,:);
ypix = ph(2,:)./ph(3,:);


% imagePoints = worldToImage(cameraParams,...
%     rod2dcm(rod),...
%     dxyz,...
%     P(1:3,:)');
% 
% xpix = imagePoints(:,1)';
% ypix = imagePoints(:,2)';