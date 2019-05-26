function [Xc1,Zc1,time] = ProposedAXZBreproj(Hgripper2base,Hcam2grid,initial,K,imgPts,Worldpts,Configuration) 
%where the Input:
%Hgripper2base      a 4x4xNumber_of_Views Matrix of the form
%                   Hgripper2base(:,:,i) = [Ri_3x2 ti_3x1;[ 0 0 0 1]] 
%                   with 
%                   i = number of the view, 
%                   Ri_3x3 the rotation matrix 
%                   ti_3x1 the translation vector.
%                   Defining the transformation from the  gripper / marker to robot base
%Hcam2grid          a 4x4xNumber_of_Views Matrix (like above)
%                   Defining the transformation of the  camerato the
%                   grid/calibration block/world
%K                  The camera intrinsic  (4x4 matrix)
%imgPts             The imgpoints corresponding to the corner points on the
%                   calibration board.(Nx2xM), where N is number of corners
%                   and M is the number of calibration images
%Worldpts           world coordinates of the corners (Nx2) matrix

%Output:
%Zc                 The transformation from gripper/marker to the camera  for the
%                   specific costfunction
%Xc                 The transformation from base to calibrtion grid/world  for the
%                   specific costfunction


tic
  n = size(Hgripper2base,3);
  A=Hgripper2base;
  B=Hcam2grid;
  Wpts = [Worldpts, zeros(size(Worldpts,1),1), ones(size(Worldpts,1),1)]';


%% using optimizer
 if strcmp(Configuration.SolAlgo,'levenberg_marquardt')
    options.Algorithm = 'levenberg-marquardt';
    options.ScaleProblem='jacobian';
    options.Display='off';
 
    % %computing for cF1
    cF1 = @(est) costFunc1(imgPts,Wpts,K,A, est);
    [x1,resnorm,residual,exitflag,output] = lsqnonlin(cF1,initial,[],[],options);

elseif strcmp(Configuration.SolAlgo,'interior_point')
    % using fmin
    options = optimoptions(@fmincon,'Algorithm','interior-point','Display','off');
    cF1 = @(est) costFunc1(imgPts,Wpts,K,A, est);
    [x1,~] = fmincon(cF1,initial,[],[],[],[],[],[],[],options);

 end

    XR1out=quat2rotm(x1(1:4));%base to world grid
    Xt1out=x1(5:7);%base to world grid
    ZR1out=quat2rotm(x1(8:11));%gripper to cam
    Zt1out=x1(12:14);%gripper to cam
    Xc1=createHT(XR1out,Xt1out); %HT base to grid
    Zc1=createHT(ZR1out,Zt1out);%HT gripper to cam
    
    time=toc;
%% defining cost functions

function [err] = costFunc1(imgPts,Wpts,K,Aarr,x)
    p=1;
    selectRow12=@(x) x(1:2,:);
    normbyZ = @(ptProj) selectRow12(bsxfun(@rdivide, ptProj, ptProj(3,:)));% % Divide with Z to complete the perspective projection and the removing the unit 1  as a resultant of the division
% err=zeros(size(imgPts,1),2);
    for t=1:length(Aarr)
              A=Aarr(:,:,t);
              imgP=imgPts(:,:,t);

%          cv2w=[1 0 0 0;0 -1 0 0;0 0 -1 0;0 0 0 1];% for blender synthetic data

               comb = @(val) norm(val)/sqrt(length(val)); 
               HT = @(val)   (   inv([[quat2rotm(val(8:11)) val(12:14)']; [0 0 0 1] ]) * A *  ([[quat2rotm(val(1:4)) val(5:7)']; [0 0 0 1] ])  )  ; % either inv(Z)*A*X  
               proj=(  normbyZ(K*((HT(x)))*Wpts) )'; %need to invert HT befor invR, if the representation change of whole inverse is followed                        

 err(:,:,t)=imgP - proj;

    end
err=reshape(permute(err,[2,1,3]),size(err,2),[])';

