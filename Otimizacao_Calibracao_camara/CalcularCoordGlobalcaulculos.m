% Matriz Intrinseca obtida da calibração
K = zeros(3,4);
k = cameraParams.IntrinsicMatrix';
K(1:3,1:3) = k;

% Calculo da Matriz Extrinseca
% Transformação do referencial da câmara para o referencial do xadrez
T_rot = zeros(4,4);
T_trans = zeros(4,4);
T_rot(1:3,1:3) = rotationMatrix;
T_trans(1:3,4) = translationVector;
T_trans(1,1)=1; T_trans(2,2)=1; T_trans(3,3)=1;
T_rot(4,4) = 1;
T_trans(4,4) = 1;
T = T_trans*T_rot;

% x = 325.250;
% y = -112.455;
% z = -151.858;
% 
% % Transformação do referencial do xadrez para o referencial mundo
% L = [0 1 0 -y
%     1 0 0 -x
%     0 0 -1 z
%     0 0 0 1];

% Matriz Extrínseca
Tf = T*L;

% Ponto da imagem a determinar nas coordenadas mundo do robô
Pi = ones(3,1);
Pi(3) = translationVector(3);
Pi(1:2,1) = imagePoints_cImg(1,:);
u = Pi(1)*Pi(3);
v = Pi(2)*Pi(3);
Po = [u v Pi(3)];

% Matriz Projeção
P = K*Tf;

% Coordenadas mundo do ponto
WP = pinv(P)*Po'