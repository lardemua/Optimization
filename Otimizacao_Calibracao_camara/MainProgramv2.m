%% MainProgram - Author -> Filipe Costa || Nmec -> 72572

clear; clc; close all;

%% Read Camera Calibration (equal for all images)
load('intrinsic_parameters.mat')

% Intrinsic matrix
intrinsics = cameraParams.IntrinsicMatrix';
intrinsics(1:3,4) = [0; 0; 0]; %homogenize

%% Image Acquisition

s{1}.filename = 'Image13.png';
s{2}.filename = 'Image4.png';
K = size(s,2);

figure('units','normalized','outerposition',[0 0 1 1]);
for k=1:K

    %load image
    s{k}.raw = imread(s{k}.filename);
    %undistort
    s{k}.undistorted = undistortImage(s{k}.raw, cameraParams, 'OutputView', 'full');
    % Detetar xadrezes na imagem
    [s{k}.image_points, s{k}.board_size] = detectCheckerboardPoints(s{k}.undistorted );
    
    %Drawing stuff
%     subplot(2,K,k+K);
    subplot(K,K+1,k*(K+1)); imshow(s{k}.undistorted); hold on
    str = ['Camara '  num2str(k)]; title(str)
    plot(s{k}.image_points(1,1),s{k}.image_points(1,2),'og')
    plot(s{k}.image_points(2:end,1),s{k}.image_points(2:end,2),'or')
end

%% Compute Chessboard 3D worl Coordinates
n_points = size(s{1}.image_points,1);
square_size = 24.73;

nx = s{1}.board_size(2); % number squares to direction x
ny = s{1}.board_size(1); % number squares to direction y
worldPoints = zeros(n_points,4);

for ix=1:nx-1
    x = (ix-1) * square_size;
    for iy=1:ny-1
        y = (iy-1) * square_size;
        worldPoints(iy+(ny-1)*(ix-1),1) = x;
        worldPoints(iy+(ny-1)*(ix-1),2) = y;
        worldPoints(iy+(ny-1)*(ix-1),3) = 0;
        worldPoints(iy+(ny-1)*(ix-1),4) = 1;
    end
end
worldPoints = worldPoints';

%% Draw 3d plot with reference systems
vplot = [];
g=0;
for g1 = 1:K
    for g2 = 1:K+1
        g=g+1;
        if g ~= g1*(K+1)
            vplot = [vplot g];
        end
    end
end
% subplot(2,K,[1 K]);
subplot(K,K+1,vplot); 
hold on; grid on; axis square;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
view(130,40); axis([-500 500 -500 500 -1500 100])
% set(gca,'CameraUpVector',[0 0 -1]);
% camorbit(gca,-110,60,'data',[0 0 1]);
% cameratoolbar('SetMode','orbit');
title('Camara positions')

plot3(worldPoints(1,:),worldPoints(2,:),worldPoints(3,:),'*');
plot3(worldPoints(1,1),worldPoints(2,1),worldPoints(3,1),'g*');

% plot3([0 1], [0 0], [0 0], 'r-');
% plot3([0 0], [0 1], [0 0], 'g-');
% plot3([0 0], [0 0], [0 1], 'b-');

%% ----- Optimization -----

%% Compute initial estimate
x0 = [];

% ---- Camera 1 - Position and Rotation
idx = 5;
dxyz = cameraParams.TranslationVectors(idx,:,:);
DCM = cameraParams.RotationMatrices(:,:,idx);
T = TFromDxyzDCM(dxyz, DCM);

x0 = [x0 T(1:3, 4)' dcm2rod(T(1:3, 1:3))];

%Draw Camara 1
orientation = DCM';
location = -dxyz * orientation;
s{1}.cam = plotCamera('Location',location,...
    'Orientation',orientation,...
    'Size',20);
s{1}.camLine = plot3([worldPoints(1,1) s{1}.cam.Location(1)],...
    [worldPoints(2,1) s{1}.cam.Location(2)],...
    [worldPoints(3,1) s{1}.cam.Location(3)], '--k');

% ---- Camera 2 - Position and Rotation
idx = 4;
dxyz = cameraParams.TranslationVectors(idx,:,:);
DCM = cameraParams.RotationMatrices(:,:,idx) ;
T = TFromDxyzDCM(dxyz, DCM);

x0 = [x0 T(1:3, 4)' dcm2rod(T(1:3, 1:3))];

%Draw Camara 2
orientation = DCM';
location = -dxyz * orientation;
s{2}.cam = plotCamera('Location',location,...
    'Orientation',orientation,...
    'Size',20);
s{2}.camLine = plot3([worldPoints(1,1) s{2}.cam.Location(1)],...
    [worldPoints(2,1) s{2}.cam.Location(2)],...
    [worldPoints(3,1) s{2}.cam.Location(3)], '--k');

%% Draw initial estimate
N = 6; %num_values_per_camera
for k=1:K
    
    dxyz = x0( (k-1)*N+1 : (k-1)*N+3 );
    rod = x0( (k-1)*N+4 : (k-1)*N+6 );
    [xpix, ypix] = points2image(dxyz, rod, intrinsics, worldPoints);
    
    %Draw intial projections
%     subplot(2,K,k+K);
    subplot(K,K+1,k*(K+1))
    s{k}.handle = plot(xpix,ypix,'+b'); hold on
    str = ['C'  num2str(k)];
    s{k}.texthandle = text(xpix(1)+0.2, ypix(1), str);
    
end

% Start optimization
f = @(x) costFunctionCamara(x, intrinsics, worldPoints, s, N, K, vplot); %define function

options = optimoptions('fminunc','Algorithm','quasi-newton'); %optimizer params
options.Display = 'iter'; 
options.MaxFunctionEvaluations = 10000;

[x, fval, exitflag, output] = fminunc(f,x0, options);

