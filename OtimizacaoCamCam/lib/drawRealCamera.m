function drawRealCamera( DCM, dxyz, worldPoints, ic, camSize )
%drawCamera - 3D rendering of the camera

orientation = DCM;
location = dxyz * orientation;
a = plotCamera('Location',location,...
    'Orientation',orientation,...
    'Size',camSize);
plot3([worldPoints(1,1) a.Location(1)],...
    [worldPoints(2,1) a.Location(2)],...
    [worldPoints(3,1) a.Location(3)], '--k');
a.Label = ['C' num2str(ic)];
a.AxesVisible = 1;
a.Color = [0 1 0];
a.Opacity = 0.01;

end