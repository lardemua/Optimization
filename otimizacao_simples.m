clc; clear; close all;

Pa_A = [-1; 3; 1];
Pb_B = [ 1; 2; 1];

figure; 
% hold on; grid on; axis equal;
% axis([-6 6 -5 10])
% plot(Pa_A(1,:), Pa_A(2,:), 'sb')
% text(Pa_A(1,:), Pa_A(2,:), 'Pa')
% 
% % Desenhar referencial A
ax_pts = [0 1 0 0; 0 0 0 1; 1 1 1 1];
% plot(ax_pts(1,1:2), ax_pts(2,1:2), 'r-')
% plot(ax_pts(1,3:4), ax_pts(2,3:4), 'g-')
% text(ax_pts(1,1), ax_pts(2,1), 'A')

% Estimativa inicial
a = 0;
dx = 3;
dy = -1;

BTA = [cos(a) -sin(a) dx
     sin(a) cos(a)  dy
        0     0     1];
      
Pb_A = BTA * Pb_B;


ax_pts_B = BTA * ax_pts;

fc = sqrt( (Pa_A(1,1) - Pb_A(1,1))^2 + ...
           (Pa_A(2,1) - Pb_A(2,1))^2);

% % Desenhar o ponto na estimativa inicial
% h1 = plot(Pb_A(1,:), Pb_A(2,:), '*r');
% h2 = text(Pb_A(1,:), Pb_A(2,:), 'Pb');
% 
% % Desenhar referencial B na estimativa inicial
% h3 = plot(ax_pts_B(1,1:2), ax_pts_B(2,1:2), 'r-');
% h4 = plot(ax_pts_B(1,3:4), ax_pts_B(2,3:4), 'g-');
% h5 = text(ax_pts_B(1,1), ax_pts_B(2,1), 'B');

f = @(x) cost_function(x);

x0 = [a dx dy];

options = optimoptions('fminunc','Algorithm','quasi-newton');
options.Display = 'iter';

[x, fval, exitflag, output] = fminunc(f,x0,options);


% for i=1:50
% 
% a = 0;
% dx = 4;
% dy = -1;
% 
% BTA = [cos(a) -sin(a) dx
%      sin(a) cos(a)  dy
%         0     0     1];
%       
% Pb_A = BTA * Pb_B;
% 
% 
% ax_pts_B = BTA * ax_pts;
% 
% fc = sqrt( (Pa_A(1,1) - Pb_A(1,1))^2 + ...
%            (Pa_A(2,1) - Pb_A(2,1))^2);
% 
% %desenhar o ponto na estimativa inicial
% set(h1, 'XData', Pb_A(1,:), 'YData', Pb_A(2,:));
% set(h2,'Position', [Pb_A(1,:) Pb_A(2,:) 0])
% 
% % %desenhar referencial B na estimativa inicial
% set(h3, 'XData', ax_pts_B(1,1:2), 'YData', ax_pts_B(2,1:2));
% set(h4, 'XData', ax_pts_B(1,3:4), 'YData', ax_pts_B(2,3:4));
% set(h5,'Position', [ax_pts_B(1,1) ax_pts_B(2,1) 0])
% 
% pause(0.3)
% 
% end