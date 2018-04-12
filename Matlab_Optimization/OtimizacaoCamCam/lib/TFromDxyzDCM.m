function T = TFromDxyzDCM(dxyz, DCM)
%return homogeneous transformation matrix from dxyz and DCM

T = zeros(4,4);
T(4,4) = 1;

T(1:3, 4) = dxyz';
T(1:3, 1:3) = DCM;
