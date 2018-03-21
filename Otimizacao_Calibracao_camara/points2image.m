function [xpix, ypix] = points2image(dxyz, rod, K, P)
%Computes the list of pixels given by the projection of P 3D points onto
%the image given the postion and intrinsics of the camera

%% Compute T matrix from dxyz and rod
T = zeros(4,4);
T(4,4) = 1; %homogeneize
T(1:3,1:3) = rod2dcm(rod);
T(1:3, 4) = dxyz';

% Calculation of the point in the image in relation to the chess reference
ph = K * T * P;

xpix = ph(1,:)./ph(3,:);
ypix = ph(2,:)./ph(3,:);
