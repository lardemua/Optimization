function [cost] = costFunction(x)


a = x(1);
dx = x(2);
dy = x(3);

Pa_A = [-1; 3; 1];
Pb_B = [ 1; 2; 1];
ax_pts = [0 1 0 0; 0 0 0 1; 1 1 1 1];



BTA = [cos(a) -sin(a) dx
    sin(a) cos(a)  dy
    0     0     1];

Pb_A = BTA * Pb_B;


ax_pts_B = BTA * ax_pts;

fc = sqrt( (Pa_A(1,1) - Pb_A(1,1))^2 + ...
    (Pa_A(2,1) - Pb_A(2,1))^2);

cost = fc;

clf
hold on; grid on; axis equal;
axis([-6 6 -5 10])
plot(-1, 3, 'ob')
text(-1+0.2, 3, 'Pa')

% Desenhar referencial A
plot([0 1], [0 0], 'r-')
plot([0 0], [0 1], 'g-')
text(0, 0, 'A')

% Desenhar o ponto na estimativa inicial
plot(Pb_A(1,:), Pb_A(2,:), '*r');
text(Pb_A(1,:)+0.2, Pb_A(2,:), 'Pb');

% Desenhar referencial B na estimativa inicial
plot(ax_pts_B(1,1:2), ax_pts_B(2,1:2), 'r-');
plot(ax_pts_B(1,3:4), ax_pts_B(2,3:4), 'g-');
text(ax_pts_B(1,1), ax_pts_B(2,1), 'B');

pause(0.1)
