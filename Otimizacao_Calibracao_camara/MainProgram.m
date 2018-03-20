%% MainProgram

% Autor: Filipe Costa  Nmec: 72572

%% Pasta onde se localizam as funções
clear; clc; close all;

%% Ler Calibração da câmara
load('intrinsic_parameters.mat')

% Matriz intrínseca
intrinsics = cameraParams.IntrinsicMatrix';
intrinsics(4,1:4) = [0 0 0 1];
intrinsics(1,4) = 0;
intrinsics(2,4) = 0;
intrinsics(3,4) = 0;

%% Aquisição de Imagem para determinar a posição do robô em relação à câmara

s{1}.filename = 'Image13.png';
s{2}.filename = 'Image4.png';
K = size(s,2);

% figure(2);  
for k=1:K

    %load image
    s{k}.raw = imread(s{k}.filename);
    %undistort
    s{k}.undistorted = undistortImage(s{k}.raw, cameraParams, 'OutputView', 'full');

    % Detetar xadrezes na imagem
    [s{k}.image_points, s{k}.board_size] = detectCheckerboardPoints(s{k}.undistorted );
    
%     %Drawing stuff
%     subplot(1,K,k); imshow(s{k}.undistorted); hold on
%     plot(s{k}.image_points(1,1),s{k}.image_points(1,2),'og')
%     plot(s{k}.image_points(2:end,1),s{k}.image_points(2:end,2),'*r')


end

%% Pontos do xadrez
n_pontos = size(s{1}.image_points,1);

square_size = 24.73;

nx = s{1}.board_size(2); % number squares to direction x
ny = s{1}.board_size(1); % number squares to direction y

points_world = zeros(n_pontos,4);

for ix=1:nx-1
    x = (ix-1) * square_size;
    for iy=1:ny-1
        y = (iy-1) * square_size;
        points_world(iy+(ny-1)*(ix-1),1) = x;
        points_world(iy+(ny-1)*(ix-1),2) = y;
        points_world(iy+(ny-1)*(ix-1),3) = 0;
        points_world(iy+(ny-1)*(ix-1),4) = 1;
    end
end

%% Otimização

figure(1);

% Estimativa inicial da posição da camara1 em relaçao ao xadrez
dx1 = -88.6236;
dy1 = -51.9176;
dz1 = -956.4222;

% Estimativa inicial da posição da camara1 em relaçao ao xadrez
dx2 = 278.0058;
dy2 = 50.8400;
dz2 = -962.7000;

% Convert direction cosine matrix to Euler-Rodrigues vector

% Rx = [1 0 0
%     0 cos(ax) -sin(ax)
%     0 sin(ax) cos(ax)];
% 
% Ry = [cos(ay) -sin(ay) 0
%     sin(ay) cos(ay) 0
%     0 0 1];
% 
% Rz = [cos(az) -sin(az) 0
%     sin(az) cos(az) 0
%     0 0 1];
% 
% DCM = Rz * Ry * Rx;

DCM = [0.9990    0.0344   -0.0296
   -0.0348    0.9993   -0.0125
    0.0291    0.0135    0.9995];

DCM = [0.9969    0.0780   -0.0130
   -0.0782    0.9969   -0.0093
    0.0123    0.0103    0.9999];

rc1 = dcm2rod( inv(DCM) );
rc2 = dcm2rod( inv(DCM) );

r11 = rc1(1); r12 = rc1(2); r13 = rc1(3);
r21 = rc2(1); r22 = rc2(2); r23 = rc2(3);

% Otimização

f = @(x) costFunctionCamara(x, intrinsics, points_world, s);

x0 = [r11 r12 r13 dx1 dy1 dz1 r21 r22 r23 dx2 dy2 dz2];

options = optimoptions('fminunc','Algorithm','quasi-newton');
options.Display = 'iter';
options.MaxFunctionEvaluations = 10000;

[x, fval, exitflag, output] = fminunc(f,x0, options);

%% Desenhar representações de pontos
r1(1) = x(1);
r1(2) = x(2);
r1(3) = x(3);
dx1 = x(4);
dy1 = x(5);
dz1 = x(6);
r2(1) = x(7);
r2(2) = x(8);
r2(3) = x(9);
dx2 = x(10);
dy2 = x(11);
dz2 = x(12);

% Transformação geométrica 1
DCM = zeros(4,4); DCM(4,4) = 1;
DCM(1:3,1:3) = rod2dcm(r1);

T = [1 0 0 dx1; 0 1 0 dy1; 0 0 1 dz1; 0 0 0 1];

C1TW = intrinsics * T * DCM;

% Calculo do ponto na imagem em relação ao ref. do xadrez
ph{1} = C1TW * points_world';
xpix{1} = ph{1}(1,:)./ph{1}(3,:);
ypix{1} = ph{1}(2,:)./ph{1}(3,:);

% Transformação geométrica 2
DCM(1:3,1:3) = rod2dcm(r2);

T = [1 0 0 dx2; 0 1 0 dy2; 0 0 1 dz2; 0 0 0 1];

C2TW = intrinsics * T * DCM;

% Calculo do ponto na imagem em relação ao ref. do xadrez
ph{2} = C2TW * points_world';
xpix{2} = ph{2}(1,:)./ph{2}(3,:);
ypix{2} = ph{2}(2,:)./ph{2}(3,:);

hold on; grid on; axis equal; xlabel('X'); ylabel('Y'); axis([0 800 0 800])

% Desenhar o pontos xadrez Imagem 1
ni=1;
plot(s{ni}.image_points(:,1), s{ni}.image_points(:,2), 'ob')
text(s{ni}.image_points(1,1)+0.2, s{ni}.image_points(1,2), 'Img1')

% Desenhar o pontos xadrez Imagem 2
ni=2;
plot(s{ni}.image_points(:,1), s{ni}.image_points(:,2), 'ok')
text(s{ni}.image_points(1,1)+0.2, s{ni}.image_points(1,2), 'Img2')

% Desenhar o ponto camara 1
cc=1;
plot(xpix{cc}, ypix{cc}, '*r');
text(xpix{cc}(1)+0.2, ypix{cc}(1), 'C1');

% Desenhar o ponto camara 2
cc=2;
plot(xpix{cc}, ypix{cc}, '*g');
text(xpix{cc}(1)+0.2, ypix{cc}(1), 'C2');
