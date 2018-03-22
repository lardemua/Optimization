function [ worldPoints ] = getWorldPoints( s, square_size )
%getWorldPoints Get Global coordinates

n_points = size(s{1}.image_points,1);

nx = s{1}.board_size(2); % number squares to direction x
ny = s{1}.board_size(1); % number squares to direction y
worldPoints = zeros(n_points,4);

for ix=1:nx-1
    x = (ix-1) * square_size;
    for iy=1:ny-1
        y = (iy-1) * square_size;
        worldPoints(iy+(ny-1)*(ix-1),1) = x;
        worldPoints(iy+(ny-1)*(ix-1),2) = y;
        worldPoints(iy+(ny-1)*(ix-1),3) = 0;
        worldPoints(iy+(ny-1)*(ix-1),4) = 1;
    end
end
worldPoints = worldPoints';

end

