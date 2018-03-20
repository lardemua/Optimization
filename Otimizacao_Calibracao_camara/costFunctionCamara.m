function [cost] = costFunctionCamara(x, intrinsics, points_world, s)


r(1) = x(1);
r(2) = x(2);
r(3) = x(3);
dx = x(4);
dy = x(5);
dz = x(6);

%% Transformação geométrica
DCM = zeros(4,4);
DCM(4,4) = 1;
DCM(1:3,1:3) = rod2dcm(r);


T = [1 0 0 dx
    0 1 0 dy
    0 0 1 dz
    0 0 0 1];

BTA = intrinsics * T * DCM;

% Calculo do ponto na imagem em relação ao ref. do xadrez
ph = BTA * points_world';

xpix = ph(1,:)./ph(3,:);
ypix = ph(2,:)./ph(3,:);

%% Distancai Euclidiana
sum_dist = 0;

ni = 1;
% for ni = 1:size(s,2)
    for n = 1:size(s{ni}.image_points,1)
        sum_dist =  sum_dist + ...
            (s{ni}.image_points(n,1) - xpix(n))^2 + ...
            (s{ni}.image_points(n,2) - ypix(n))^2;
    end
% end

fc = sqrt(sum_dist);

cost = fc;

clf
hold on; grid on; axis equal; %view(100,40);
xlabel('X'); ylabel('Y');% zlabel('Z');
axis([0 800 0 800])

% Desenhar o ponto a
plot(s{ni}.image_points(:,1), s{ni}.image_points(:,2), 'ob')
text(s{ni}.image_points(1,1)+0.2, s{ni}.image_points(1,2), 'Pa')

% Desenhar o ponto b
plot(xpix, ypix, '*r');
text(xpix(1)+0.2, ypix(1), 'Pb');

pause(0.01)
