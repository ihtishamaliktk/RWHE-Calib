% This file loads all the relevant data and computes the Hand-eye
% calibration using different methods and provides statistical analysis
clear
close all
%% Add Currentand dependent Paths
AddPaths

%% Load Configuration
Config=H2EConfiguration;

%% Load Dataset
DatasetName='kuka_2'; % CS_synthetic_1,'kuka_%d','Custom'      NOTE: %d should be replaced with the dataset number
calibImgPath=[pwd,'\Datasets\',DatasetName,'\'];
squareSize=getSquareSize4dataset(DatasetName); % in meters   
robotPosesOrig=load([pwd,'\Datasets\',DatasetName,'\RobotPoses.mat']); % position should be in meters
%% Compute Camera Intrinsics
[cameraParams, UndistoredImgNPts,OrigImgNPts,imagesUsed, camPoses, estimationErrors,Worldpts] = CalibrateIntrinsics(calibImgPath, squareSize,Config);
% figure, showExtrinsics(cameraParams);  % 
figure, showExtrinsics(cameraParams,'patternCentric'); 

UndistortArray=reshape(UndistoredImgNPts(1:3:end,1),1,1,[]);
figure,montage(UndistortArray),title('undistorted img')

%% Camera poses
poses = createPosesFromCameraParams(cameraParams);% default poses without the bundle adjustment from intrinsics
%% pose conformity for ax=xb approach
format='HTvector';
robotPoses=robotPosesOrig.handposes;%robots end effector position

[tcpPoseInBaseCoords,gridPoseInCameraCoords,cameraPoseInGridCoords]=poserRepresentationConformability(robotPoses,camPoses,imagesUsed,'DataFormat',format);%confirm the poses to Homogenous transformation matrices

%% induce noise in TcpPose
if ~isempty(Config.Nr) | ~isempty(Config.Nt)
    tcpPoseInBaseCoords=inducePoseNoise(tcpPoseInBaseCoords,Config);
end

%% AX=XB approaches
[handEyeHT.Tsai,time.Tsai] = method.Lazax_HandeyeTSAI(tcpPoseInBaseCoords, cameraPoseInGridCoords); %pose of camera relative to the grid
[handEyeHT.HoraudDornaika, time.HoraudDornaika] = method.inria_calibration(tcpPoseInBaseCoords, gridPoseInCameraCoords);%pose of grid relative to the camera (Hcam2grid2)
[handEyeHT.ParkMartin, time.ParkMartin] = method.navy_calibration(tcpPoseInBaseCoords, gridPoseInCameraCoords); %pose of grid relative to the camera (Hcam2grid2)



%% extracting A,B  for milli shah implementations
% NOTE:: constrain the following fucntion so it extracts poses by the
% chosen calibration images
% milli shah implementation denotes relative camera poses as Bij and relative robotic arm tcp poses as Aij
%concatenating the 3d matrices as 2D matrices A=[A1 A2 ...An]  ,B=[B1 B2 ...Bn]
for p=1:length(tcpPoseInBaseCoords)
    BasePoseIntcpCoords(:,:,p)=invertHT(tcpPoseInBaseCoords(:,:,p));
end
  tcpPoseInBaseCoords_AA = reshape(tcpPoseInBaseCoords,4,[],1);  gridPoseInCameraCoords_BB = reshape(gridPoseInCameraCoords,4,[],1);
  BasePoseIntcpCoords_AA = reshape(BasePoseIntcpCoords,4,[],1); 
  cameraPoseInGridCoords_BB = reshape(cameraPoseInGridCoords,4,[],1);
  




%% AX=ZB Approaches
[base2grid.Li,handEyeHT.Li,time.Li] = method.li(BasePoseIntcpCoords_AA,gridPoseInCameraCoords_BB) ; %pose of grid relative to the camera (Hcam2grid2)
[base2grid.Shah,handEyeHT.Shah,time.Shah] = method.HandeyeShah(BasePoseIntcpCoords_AA,gridPoseInCameraCoords_BB) ; %pose of grid relative to the camera (Hcam2grid2)


%% Direct optimization bsaed on poses ( Milli Shahs illustrations adopted )

%AX=XB direct iterative 
initial=[ rotm2quat(handEyeHT.Li(1:3,1:3)) ,handEyeHT.Li(1:3,4)'];
[handEyeHT.Xc1,handEyeHT.Xc2,time.Xc1,time.Xc2] = method.ProposedAXXB(tcpPoseInBaseCoords,gridPoseInCameraCoords,initial) ; %gridPoseInCameraCoords mean Hcam2grid

Config.SolAlgo='levenberg_marquardt';
initial=[ rotm2quat(base2grid.Li(1:3,1:3)) base2grid.Li(1:3,4)'  rotm2quat(handEyeHT.Li(1:3,1:3)) ,handEyeHT.Li(1:3,4)'];
[~,handEyeHT.TabbZc1,base2grid.TabbZc2,handEyeHT.TabbZc2,time.TabbZc1,time.TabbZc2] = method.AXZB(BasePoseIntcpCoords,gridPoseInCameraCoords,initial,Config) ;
% visualizeProjection(base2grid_Zc2,handEyeHT.Zc2,imgPts,Worldpts,BasePoseIntcpCoords,UndistoredImgNPts(:,1),KM,gridPoseInCameraCoords,'AXZB');%OrigImgNPts


%% Reprojection based Approach
K= cameraParams.IntrinsicMatrix';% matlabs interpretation of intrinsic matrix is different than general approach
KM=[K [0;0;0]];
temp=UndistoredImgNPts(:,2);imgPts =cat(3,temp{:});
 

%AX=XB  iterative reprojection based approach
initial=[ rotm2quat(handEyeHT.TabbZc2(1:3,1:3)) ,handEyeHT.TabbZc2(1:3,4)'];
[handEyeHT.RX,time.RX] = method.ProposedAXXBreproj(tcpPoseInBaseCoords,gridPoseInCameraCoords,initial,KM,imgPts,Worldpts,Config) ; %gridPoseInCameraCoords mean Hcam2grid
% visualizeProjection([],handEyeHT.rXc1,imgPts,Worldpts,tcpPoseInBaseCoords,UndistoredImgNPts(:,1),KM,gridPoseInCameraCoords,'AXXB');%OrigImgNPts


%AX=ZB  iterative reprojection based approach
%reprojection based method without camera  intrinsic optimisation for AX=ZB
initial=[ rotm2quat(base2grid.TabbZc2(1:3,1:3)) base2grid.TabbZc2(1:3,4)'  rotm2quat(handEyeHT.TabbZc2(1:3,1:3)) ,handEyeHT.TabbZc2(1:3,4)'];
[base2grid.Tabbrp1,handEyeHT.Tabbrp1,time.Tabbrp1] = method.AXZBreproj(BasePoseIntcpCoords,gridPoseInCameraCoords,initial,KM,imgPts,Worldpts,Config) ;
temp=UndistoredImgNPts(:,2);imgPts =cat(3,temp{:});


[base2grid.RZ,handEyeHT.RZ,time.RZ] = method.ProposedAXZBreprojC2(BasePoseIntcpCoords,gridPoseInCameraCoords,initial,KM,imgPts,Worldpts,Config) ;


%% tabulate handye calibration values
tabulatePoses(handEyeHT)

%% Computing Errors
if strcmp(DatasetName(1:end-1),'CS_synthetic_')
GT=[1.0000    0.0000    0.0000    0.0000
    0.0000   -1.0000    0.0000    0.0000
    0.0001    0.0001    0.0000    0.0000
    0         0         0         1.0000];
else
    GT=[];
end
[methodsNames,Hhand2eye,Hbase2grid,timearr]=formatPoses(handEyeHT,base2grid,time);
 [form,errs]=computeErrors(tcpPoseInBaseCoords,Hhand2eye,Hbase2grid,gridPoseInCameraCoords,GT,imgPts,Worldpts,KM);
tabulateErrors(methodsNames,form,errs,timearr)

%% Visualize : Uncomment to visualize
warning ('off','all');
visualizeProjection(base2grid.RZ,(handEyeHT.RZ),imgPts,Worldpts,BasePoseIntcpCoords,UndistoredImgNPts(:,1),KM,gridPoseInCameraCoords,'AXZB');%OrigImgNPts
warning ('on','all');

