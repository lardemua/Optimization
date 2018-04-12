function dispResults( cameraParams, x, K, N )
%dispResults Display results and compare

ir = cameraParams.IntrinsicMatrix';
ir(1:3,4) = [0; 0; 0]; %homogenize

idx=1;
dxyz = cameraParams.TranslationVectors(idx,:,:);
DCM = cameraParams.RotationMatrices(:,:,idx) ;
Tr{1} = TFromDxyzDCM(dxyz, DCM);

idx=2;
dxyz = cameraParams.TranslationVectors(idx,:,:);
DCM = cameraParams.RotationMatrices(:,:,idx) ;
Tr{2} = TFromDxyzDCM(dxyz, DCM);

idx=4;
dxyz = cameraParams.TranslationVectors(idx,:,:);
DCM = cameraParams.RotationMatrices(:,:,idx) ;
Tr{3} = TFromDxyzDCM(dxyz, DCM);

disp('>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<');
disp('>>>>>>>>>> Resultados <<<<<<<<<<<');
disp('>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<');

for k=1:K
    
    str = ['---> Camera '  num2str(k) ' <---'];
    dxyz = x( (k-1)*N+1 : (k-1)*N+3 );
    rod = x( (k-1)*N+4 : (k-1)*N+6 ); DCM = rod2dcm(rod);
    intrinsic_vector = x( (k-1)*N+7 : (k-1)*N+10 );
    intrinsics = vectorToInterinsic( intrinsic_vector );
    T = TFromDxyzDCM(dxyz, DCM);
    
    disp(str); 
    disp('-- Transform Matrix --');
    disp(num2str(T));
    disp('-- Intrinsic Matrix --');
    disp(num2str(intrinsics));
    
end
disp('>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<');
disp('>>>>>>>>>> Comparações <<<<<<<<<<');
disp('>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<');

for k=1:K
    
    str = ['---> Camera '  num2str(k) ' <---'];
    dxyz = x( (k-1)*N+1 : (k-1)*N+3 );
    rod = x( (k-1)*N+4 : (k-1)*N+6 ); 
    DCM = rod2dcm(rod);
    intrinsic_vector = x( (k-1)*N+7 : (k-1)*N+10 );
    intrinsics = vectorToInterinsic( intrinsic_vector );
    T = TFromDxyzDCM(dxyz, DCM);   
    
    disp(str); 
    disp('-- Transform Matrix --');
    erro = Tr{k} - T;
    disp(num2str(erro));
    disp('-- Intrinsic Matrix --');
    erroi = ir - intrinsics;
    disp(num2str(erroi));
    
end


end

