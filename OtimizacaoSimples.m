%% Calibração Câmara-Câmara
clc; clear; close all;

%% Criar uma figura
figure;

% %% Otimização Simples 2D
% % Pontos conhecidos
% Pa_A = [-1; 3; 1];
% Pb_B = [ 1; 2; 1];
%
% % Referencial A
% ax_pts = [0 1 0 0;
%           0 0 0 1;
%           1 1 1 1
%           ];
%
% % Estimativa inicial da posição do ref. B em relação ao ref. A
% a = 0;
% dx = 3;
% dy = -1;
%
% BTA = [cos(a) -sin(a) dx
%        sin(a) cos(a)  dy
%         0     0     1];
%
% % Calculo do ponto b em relação ao ref. A
% Pb_A = BTA * Pb_B;
%
% % Referencial B
% ax_pts_B = BTA * ax_pts;
%
% % Função de custo
% % fc = sqrt( (Pa_A(1,1) - Pb_A(1,1))^2 + ...
% %            (Pa_A(2,1) - Pb_A(2,1))^2);
%
% f = @(x) costFunction(x);
%
% x0 = [a dx dy];
%
% options = optimoptions('fminunc','Algorithm','quasi-newton');
% options.Display = 'iter';
%
% [x, fval, exitflag, output] = fminunc(f,x0,options);

% %% Otimização Simples 3D
% 
% % Estimativa inicial da posição do ref. B em relação ao ref. A
% ax = 0;
% ay = 0;
% az = 0;
% dx = 3;
% dy = -1;
% dz = -8;
% 
% % Convert direction cosine matrix to Euler-Rodrigues vector
% 
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
% 
% r = dcm2rod( DCM );
% 
% r1 = r(1); r2 = r(2); r3 = r(3);
% 
% % Otimização
% 
% f = @(x) costFunction3D(x);
% 
% x0 = [r1 r2 r3 dx dy dz];
% 
% options = optimoptions('fminunc','Algorithm','quasi-newton');
% options.Display = 'iter';
% 
% [x, fval, exitflag, output] = fminunc(f,x0,options);

%% Otimização 3D, N pontos

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

f = @(x) costFunction3DnPoints(x);

x0 = [r1 r2 r3 dx dy dz];

options = optimoptions('fminunc','Algorithm','quasi-newton');
options.Display = 'iter';

[x, fval, exitflag, output] = fminunc(f,x0,options);


