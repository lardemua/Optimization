function [cost] = costFunctionCamara(x, intrinsics, points_world, s)


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

%% Transformação geométrica 1
DCM1 = zeros(4,4);
DCM1(4,4) = 1;
DCM1(1:3,1:3) = rod2dcm(r1);


T = [1 0 0 dx1
    0 1 0 dy1
    0 0 1 dz1
    0 0 0 1];

C1TW = intrinsics * T * DCM1;

% Calculo do ponto na imagem em relação ao ref. do xadrez
ph{1} = C1TW * points_world';

xpix{1} = ph{1}(1,:)./ph{1}(3,:);
ypix{1} = ph{1}(2,:)./ph{1}(3,:);

%% Transformação geométrica 2
DCM2 = zeros(4,4);
DCM2(4,4) = 1;
DCM2(1:3,1:3) = rod2dcm(r2);


T = [1 0 0 dx2
    0 1 0 dy2
    0 0 1 dz2
    0 0 0 1];

C2TW = intrinsics * T * DCM2;

% Calculo do ponto na imagem em relação ao ref. do xadrez
ph{2} = C2TW * points_world';

xpix{2} = ph{2}(1,:)./ph{2}(3,:);
ypix{2} = ph{2}(2,:)./ph{2}(3,:);

%% Otimizacao Distancia
sum_dist = 0;

for ni = 1:size(s,2)
    sum_dist =  sum_dist + ...
        sum((s{ni}.image_points(:,1) - xpix{ni}(:)).^2 + ...
        (s{ni}.image_points(:,2) - ypix{ni}(:)).^2);
end

fc = sqrt(sum_dist);
cost = fc;

% %% Desenhar representações de pontos
% clf
% hold on; grid on; axis equal; %view(100,40);
% xlabel('X'); ylabel('Y');% zlabel('Z');
% axis([0 800 0 800])
% 
% % Desenhar o pontos xadrez Imagem 1
% ni=1;
% plot(s{ni}.image_points(:,1), s{ni}.image_points(:,2), 'ob')
% text(s{ni}.image_points(1,1)+0.2, s{ni}.image_points(1,2), 'Img1')
% 
% % Desenhar o pontos xadrez Imagem 2
% ni=2;
% plot(s{ni}.image_points(:,1), s{ni}.image_points(:,2), 'ok')
% text(s{ni}.image_points(1,1)+0.2, s{ni}.image_points(1,2), 'Img2')
% 
% % Desenhar o ponto camara 1
% cc=1;
% plot(xpix{cc}, ypix{cc}, '*r');
% text(xpix{cc}(1)+0.2, ypix{cc}(1), 'C1');
% 
% % Desenhar o ponto camara 2
% cc=2;
% plot(xpix{cc}, ypix{cc}, '*g');
% text(xpix{cc}(1)+0.2, ypix{cc}(1), 'C2');
% 
% pause(0.01)
