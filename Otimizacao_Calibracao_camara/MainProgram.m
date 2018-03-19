%% MainProgram

% Autor: Filipe Costa  Nmec: 72572

%% Pasta onde se localizam as funções
clear; clc; close all;

%% Ler Calibração da câmara
load('intrinsic_parameters.mat')

% Matriz intrínseca
M_intrinsic = cameraParams.IntrinsicMatrix;

%% Aquisição de Imagem para determinar a posição do robô em relação à câmara

s{1}.filename = 'Image13.png';
s{2}.filename = 'Image4.png';
K = size(s,2);

figure(1);  
for k=1:K

    %load image
    s{k}.raw = imread(s{k}.filename);
    %undistort
    s{k}.undistorted = undistortImage(s{k}.raw, cameraParams, 'OutputView', 'full');

    % Detetar xadrezes na imagem
    [s{k}.image_points, s{k}.board_size] = detectCheckerboardPoints(s{k}.undistorted );

    %Drawing stuff
    subplot(1,K,k); imshow(s{k}.undistorted); hold on
    plot(s{k}.image_points(1,1),s{k}.image_points(1,2),'og')
    plot(s{k}.image_points(2:end,1),s{k}.image_points(2:end,2),'*r')


end

%% Pontos do xadrez
n_pontos = size(s{1}.image_points,1);

square_size = 24.73;

nx = s{1}.board_size(2); % number squares to direction x
ny = s{1}.board_size(1); % number squares to direction y

points_world = zeros(n_pontos,3);

for iy=1:ny-1
    y = (iy-1) * square_size;
    for ix=1:nx-1
        x = (ix-1) * square_size;
        points_world(ix+(nx-1)*(iy-1),1) = x;
        points_world(ix+(nx-1)*(iy-1),2) = y;
        points_world(ix+(nx-1)*(iy-1),3) = 0;
    end
end

%% Otimização

% Estimativa inicial da posição do ref. B em relação ao ref. A
ax = 0;
ay = 0;
az = 0;
dx = 3;
dy = -1;
dz = -8;

% Convert direction cosine matrix to Euler-Rodrigues vector

Rx = [1 0 0
    0 cos(ax) -sin(ax)
    0 sin(ax) cos(ax)];

Ry = [cos(ay) -sin(ay) 0
    sin(ay) cos(ay) 0
    0 0 1];

Rz = [cos(az) -sin(az) 0
    sin(az) cos(az) 0
    0 0 1];

DCM = Rz * Ry * Rx;

r = dcm2rod( DCM );

r1 = r(1); r2 = r(2); r3 = r(3);

% Otimização

f = @(x) costFunctionCamara(x);

x0 = [r1 r2 r3 dx dy dz];

options = optimoptions('fminunc','Algorithm','quasi-newton');
options.Display = 'iter';

[x, fval, exitflag, output] = fminunc(f,x0,options);






