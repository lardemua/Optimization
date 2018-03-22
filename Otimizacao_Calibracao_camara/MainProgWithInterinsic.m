%% MainProgram - Author -> Filipe Costa || Nmec -> 72572

clear; clc; close all;

%% Read Camera Calibration (equal for all images)
load('intrinsic_parameters_pg.mat')

% Intrinsic matrix
intrinsics = cameraParams.IntrinsicMatrix';
intrinsics(1:3,4) = [0; 0; 0]; %homogenize

%% Image Acquisition

s{1}.filename = '0001.png';
s{2}.filename = '0002.png';
s{3}.filename = '0004.png';
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
square_size = 105; %24.73;
worldPoints = getWorldPoints( s, square_size );

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
view(3,-88); axis([-1500 1500 -1500 1500 -3000 100])
title('Camera positions')

plot3(worldPoints(1,:),worldPoints(2,:),worldPoints(3,:),'s');
plot3(worldPoints(1,1),worldPoints(2,1),worldPoints(3,1),'gs');

exsize = 200;
plot3([0 exsize], [0 0], [0 0], 'r-');
plot3([0 0], [0 exsize], [0 0], 'g-');
plot3([0 0], [0 0], [-exsize 0], 'b-');

%% ----- Optimization -----

%% Compute initial estimate
x0 = []; camSize = 50;

% ---- Camera 1 - Position and Rotation
idx = 1;
dxyz = cameraParams.TranslationVectors(idx,:,:);
DCM = cameraParams.RotationMatrices(:,:,idx);
T = TFromDxyzDCM(dxyz, DCM);
intrinsic_vector = interinsicToVector( intrinsics );

x0 = [x0 T(1:3, 4)' dcm2rod(T(1:3, 1:3)) intrinsic_vector];

% Draw
ic = 1;
s = drawCamera( DCM, dxyz, worldPoints, s, ic, camSize );

% ---- Camera 2 - Position and Rotation
idx = 2;
dxyz = cameraParams.TranslationVectors(idx,:,:);
DCM = cameraParams.RotationMatrices(:,:,idx) ;
T = TFromDxyzDCM(dxyz, DCM);
intrinsic_vector = interinsicToVector( intrinsics );

x0 = [x0 T(1:3, 4)' dcm2rod(T(1:3, 1:3)) intrinsic_vector];

% Draw
ic = 2;
s = drawCamera( DCM, dxyz, worldPoints, s, ic, camSize );

% ---- Camera 3 - Position and Rotation
idx = 4;
dxyz = cameraParams.TranslationVectors(idx,:,:);
DCM = cameraParams.RotationMatrices(:,:,idx) ;
T = TFromDxyzDCM(dxyz, DCM);
intrinsic_vector = interinsicToVector( intrinsics );

x0 = [x0 T(1:3, 4)' dcm2rod(T(1:3, 1:3)) intrinsic_vector];

% Draw
ic = 3;
s = drawCamera( DCM, dxyz, worldPoints, s, ic, camSize );

%% Draw initial estimate
N = 10; %num_values_per_camera
for k=1:K
    
    dxyz = x0( (k-1)*N+1 : (k-1)*N+3 );
    rod = x0( (k-1)*N+4 : (k-1)*N+6 );
    intrinsic_vector = x0( (k-1)*N+7 : (k-1)*N+10 );
    intrinsics = vectorToInterinsic( intrinsic_vector );
    [xpix, ypix] = points2image(dxyz, rod, intrinsics, worldPoints);
    
    %Draw intial projections
    subplot(K,K+1,k*(K+1))
    s{k}.handle = plot(xpix,ypix,'+m'); hold on
    str = ['C'  num2str(k)];
    s{k}.texthandle = text(xpix(1)+0.2, ypix(1), str);
    
end

%% Start optimization
f = @(x) costFunctionCamaraInterinsic(x, worldPoints, s, N, K, vplot); %define function

options = optimoptions('fminunc','Algorithm','quasi-newton'); %optimizer params
options.Display = 'iter'; 
options.MaxFunctionEvaluations = 10000;

[x, fval, exitflag, output] = fminunc(f,x0, options);

%% Display Results
dispResults( cameraParams, x, K, N );












