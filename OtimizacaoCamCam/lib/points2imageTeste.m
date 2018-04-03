function [xpix, ypix] = points2imageTeste(dxyz, rod, K, P, cameraParams)
%Computes the list of pixels given by the projection of P 3D points onto
%the image given the postion and intrinsics of the camera

%% Compute T matrix from dxyz and rod
T = zeros(4,4);
T(4,4) = 1; %homogeneize
T(1:3,1:3) = rod2dcm(rod);
T(1:3, 4) = dxyz';

% Calculation of the point in the image in relation to the chess reference

xpix = [];
ypix = [];

fx = K(1,1);
fy = K(2,2);
cx = K(1,3);
cy = K(2,3);

k1 = cameraParams.RadialDistortion(1,1);
k2 = cameraParams.RadialDistortion(1,2);
p1 = cameraParams.TangentialDistortion(1,1);
p2 = cameraParams.TangentialDistortion(1,2);

for i=1:size(P,2)
    
    xyz = P(1:3,i)' * T(1:3,1:3) + T(1:3,4)';
    
    xl = xyz(1)/ xyz(3);
    yl = xyz(2)/ xyz(3);
    
    r_square = xl^2 + yl^2;
    
    xll = xl * (1 + k1 * r_square + k2 * r_square^2) + 2 * p1 * xl * yl + p2 * (r_square + 2 * xl^2);
    yll = yl * (1 + k1 * r_square + k2 * r_square^2) + p1 * (r_square + 2 * yl^2) + 2 * p2 * xl * yl;
    
    
    u = fx * xll + cx;
    v = fy * yll + cy;
    
    xpix = [xpix u];
    ypix = [ypix v];
    
end