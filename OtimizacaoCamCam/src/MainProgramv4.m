%% MainProgram - Author -> Filipe Costa || Nmec -> 72572

clear; clc; close all;

addpath('../data')
addpath('../../images')
addpath('../lib')

%% Read Camera Calibration (equal for all images)
load('intrinsic_parameters1.mat')

% Intrinsic matrix
intrinsics = cameraParams.IntrinsicMatrix';
intrinsics(1:3,4) = [0; 0; 0]; %homogenize


%% Image Acquisition

s{1}.filename = '0001.jpg';
s{2}.filename = '0014.jpg';
s{3}.filename = '0024.jpg';

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
    subplot(K,K+1,k*(K+1));
    imshow(s{k}.undistorted); hold on
    str = ['Camera '  num2str(k)]; title(str)
    plot(s{k}.image_points(1,1),s{k}.image_points(1,2),'ob')
    plot(s{k}.image_points(2:end,1),s{k}.image_points(2:end,2),'og')
end

%% Compute Chessboard 3D worl Coordinates
square_size = 105;
worldPoints = getWorldPoints( s, square_size );
% worldPoints = worldPoints(:,1);

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
subplot(K,K+1,vplot); 
hold on; grid on; axis square;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
view(3,-88); axis([-1500 1500 -1500 1500 -5000 3000])
title('Camera positions')

plot3(worldPoints(1,:),worldPoints(2,:),worldPoints(3,:),'s');
plot3(worldPoints(1,1),worldPoints(2,1),worldPoints(3,1),'gs');

exsize = 200;
plot3([0 exsize], [0 0], [0 0], 'r-');
plot3([0 0], [0 exsize], [0 0], 'g-');
plot3([0 0], [0 0], [0 exsize], 'b-');

%% ----- Optimization -----

%% Compute initial estimate
x0 = []; camSize = 100;

% ---- Camera 1 - Position and Rotation
idx = 1;
dxyz = cameraParams.TranslationVectors(idx,:,:);
DCM = cameraParams.RotationMatrices(:,:,idx);
T = TFromDxyzDCM(dxyz, DCM);

% T = [-0.8910   -0.4472   -0.0785 0.4517
%    -0.4479    0.8940   -0.0093   -0.4057
%     0.0744    0.0269   -0.9969    2.0376
%      0 0 0 1]; 

%  CalibResults.m
% omc_1 = [ 7.054946e-01 ; -2.980546e+00 ; -1.386317e-02 ];
% Tc_1  = [ 4.516642e-01 ; -4.057450e-01 ; 2.037621e+00 ];


%  T = [-0.7115   -0.4078    0.5723  0.4517
%    -0.4024    0.9041    0.1439 -0.4057
%    -0.5761   -0.1280   -0.8073 2.0376
%    0 0 0 1];

intrinsic_vector = interinsicToVector( intrinsics );

x0 = [x0 T(1:3, 4)' dcm2rod(T(1:3, 1:3)) intrinsic_vector];

% Draw
ic = 1;
s = drawCamera( DCM', -dxyz, worldPoints, s, ic, camSize );

% ---- Camera 2 - Position and Rotation
idx = 14;
dxyz = cameraParams.TranslationVectors(idx,:,:);
DCM = cameraParams.RotationMatrices(:,:,idx);
T = TFromDxyzDCM(dxyz, DCM);
intrinsic_vector = interinsicToVector( intrinsics );

x0 = [x0 T(1:3, 4)' dcm2rod(T(1:3, 1:3)) intrinsic_vector];

% % Draw
ic = 2;
s = drawCamera( DCM', -dxyz, worldPoints, s, ic, camSize );

% ---- Camera 3 - Position and Rotation
idx = 24;
dxyz = cameraParams.TranslationVectors(idx,:,:);
DCM = cameraParams.RotationMatrices(:,:,idx);
T = TFromDxyzDCM(dxyz, DCM);
intrinsic_vector = interinsicToVector( intrinsics );

x0 = [x0 T(1:3, 4)' dcm2rod(T(1:3, 1:3)) intrinsic_vector];

% Draw
ic = 3;
s = drawCamera( DCM', -dxyz, worldPoints, s, ic, camSize );

%% Draw initial estimate
N = 10; %num_values_per_camera
for k=1:K
    
    dxyz = x0( (k-1)*N+1 : (k-1)*N+3 );
    rod = x0( (k-1)*N+4 : (k-1)*N+6 );
    intrinsic_vector = x0( (k-1)*N+7 : (k-1)*N+10 );
    intrinsics = vectorToInterinsic( intrinsic_vector );
    [xpix, ypix] = points2imageTeste(dxyz, rod, intrinsics, worldPoints, cameraParams);
    
    %Draw intial projections
    subplot(K,K+1,k*(K+1))
    s{k}.handle = plot(xpix,ypix,'+m'); hold on
    str = ['C'  num2str(k)];
    s{k}.texthandle = text(xpix(1)+0.2, ypix(1), str);
    
end

return
%% Start optimization
f = @(x) costFunctionCamaraInterinsicV2(x, worldPoints, s, N, K, vplot, cameraParams); %define function

options = optimoptions('fminunc','Algorithm','quasi-newton'); %optimizer params
options.Display = 'iter'; 
options.MaxFunctionEvaluations = 10000;

[x, fval, exitflag, output] = fminunc(f,x0, options);

%% Display Results
% dispResults( cameraParams, x, K, N );


% ---- Camera 1
idx = 1;
dxyz = cameraParams.TranslationVectors(idx,:,:);
DCM = cameraParams.RotationMatrices(:,:,idx) ;
intrinsic_vector = interinsicToVector( intrinsics );
ic=1;
drawRealCamera( DCM', -dxyz, worldPoints, ic, camSize );

% ---- Camera 2
idx = 14;
dxyz = cameraParams.TranslationVectors(idx,:,:);
DCM = cameraParams.RotationMatrices(:,:,idx) ;
intrinsic_vector = interinsicToVector( intrinsics );
ic=2;
drawRealCamera( DCM', -dxyz, worldPoints, ic, camSize );

% ---- Camera 1
idx = 24;
dxyz = cameraParams.TranslationVectors(idx,:,:);
DCM = cameraParams.RotationMatrices(:,:,idx) ;
intrinsic_vector = interinsicToVector( intrinsics );
ic=3;
drawRealCamera( DCM', -dxyz, worldPoints, ic, camSize );





