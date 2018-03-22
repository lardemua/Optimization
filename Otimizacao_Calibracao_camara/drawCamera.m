function [ s ] = drawCamera( DCM, dxyz, worldPoints, s, ic, camSize )
%drawCamera - 3D rendering of the camera

orientation = DCM;
location = dxyz * orientation;
s{ic}.cam = plotCamera('Location',location,...
    'Orientation',orientation,...
    'Size',camSize);
s{ic}.camLine = plot3([worldPoints(1,1) s{ic}.cam.Location(1)],...
    [worldPoints(2,1) s{ic}.cam.Location(2)],...
    [worldPoints(3,1) s{ic}.cam.Location(3)], '--k');
s{ic}.cam.Label = ['C' num2str(ic)];
s{ic}.cam.AxesVisible = 1;


end

