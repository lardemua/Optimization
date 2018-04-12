function [cost] = costFunction3DnPoints(x)


r(1) = x(1);
r(2) = x(2);
r(3) = x(3);
dx = x(4);
dy = x(5);
dz = x(6);

% Pontos conhecidos
Pa_A = [-1 3 0; 0 3 0; 1 3 0; 2 3 0
    -1 4 0; 0 4 0; 1 4 0; 2 4 0
    -1 5 1; 0 5 1; 1 5 1; 2 5 1]';
Pa_A(4,1:end) = 1;
Pb_B = [ 1 2 0; 2 2 0; 3 2 0; 4 2 0
    1 3 0; 2 3 0; 3 3 0; 4 3 0
    1 4 1; 2 4 1; 3 4 1; 4 4 1]';
Pb_B(4,1:end) = 1;

% Referencial A
ax_pts = [0 1 0 0 0 0
    0 0 0 1 0 0
    0 0 0 0 0 1
    1 1 1 1 1 1
    ];
DCM = zeros(4,4);
DCM(4,4) = 1;
DCM(1:3,1:3) = rod2dcm(r);


T = [1 0 0 dx
    0 1 0 dy
    0 0 1 dz
    0 0 0 1];

BTA = T * DCM;

% Calculo do ponto b em relação ao ref. A
Pb_A = BTA * Pb_B;

% Referencial B
ax_pts_B = BTA * ax_pts;

sum_dist = 0;
for n = 1:size(Pa_A,2)
    sum_dist =  sum_dist + ...
        (Pa_A(1,n) - Pb_A(1,n))^2 + ...
        (Pa_A(2,n) - Pb_A(2,n))^2 + ...
        (Pa_A(3,n) - Pb_A(3,n))^2;
end

fc = sqrt(sum_dist);

cost = fc;

clf
hold on; grid on; axis equal; view(100,40);
xlabel('X'); ylabel('Y'); zlabel('Z');
axis([-3 4 -1 6 -4 4])

% Desenhar o ponto a
plot3(Pa_A(1,:), Pa_A(2,:), Pa_A(3,:), 'ob')
text(Pa_A(1,:)+0.2, Pa_A(2,:), Pa_A(3,:), 'Pa')

% Desenhar referencial A
plot3(ax_pts(1,1:2), ax_pts(2,1:2), ax_pts(3,1:2), 'r-')
plot3(ax_pts(1,3:4), ax_pts(2,3:4), ax_pts(3,3:4), 'g-')
plot3(ax_pts(1,5:6), ax_pts(2,5:6), ax_pts(3,5:6), 'b-')
text(ax_pts(1,1), ax_pts(2,1), ax_pts(3,1), 'A')

% Desenhar o ponto b
plot3(Pb_A(1,:), Pb_A(2,:), Pb_A(3,:), '*r');
text(Pb_A(1,:)+0.2, Pb_A(2,:), Pb_A(3,:), 'Pb');

% Desenhar referencial B
plot3(ax_pts_B(1,1:2), ax_pts_B(2,1:2), ax_pts_B(3,1:2), 'r-')
plot3(ax_pts_B(1,3:4), ax_pts_B(2,3:4), ax_pts_B(3,3:4), 'g-')
plot3(ax_pts_B(1,5:6), ax_pts_B(2,5:6), ax_pts_B(3,5:6), 'b-')
text(ax_pts_B(1,1), ax_pts_B(2,1), ax_pts_B(3,1), 'B')

pause(0.01)
