function [Xc1,time] = ProposedAXXBreproj(Hbase2gripper,Hcam2grid,initial,K,imgPts,Worldpts,Configuration) 
%where the Input:
%Hbase2gripper      a 4x4xNumber_of_Views Matrix of the form
%                   Hbase2gripper(:,:,i) = [Ri_3x3 ti_3x1;[ 0 0 0 1]] 
%                   with 
%                   i = number of the view, 
%                   Ri_3x3 the rotation matrix 
%                   ti_3x1 the translation vector.
%                   Defining the transformation from the robot base to gripper / marker
%Hcam2grid          a 4x4xNumber_of_Views Matrix (like above)
%                   Defining the transformation of the  camerato the
%                   grid/calibration block/world
%K                  The camera intrinsic  (4x4 matrix)
%imgPts             The imgpoints corresponding to the corner points on the
%                   calibration board.(Nx2xM), where N is number of corners
%                   and M is the number of calibration images
%Worldpts           world coordinates of the corners (Nx2) matrix

%Output:
%Xc                 The transformation from base to calibrtion grid/world  for the
%                   specific costfunction



tic
  A = [];
  B=  [];
  n = size(Hbase2gripper,3);
  Wpts = [Worldpts, zeros(size(Worldpts,1),1), ones(size(Worldpts,1),1)]';

  for k=1:n-1
      A(:,:,k)= inv(Hbase2gripper(:,:,k+1))* Hbase2gripper(:,:,k); % relative poses for Gripper
%     B(:,:,k)= Hcam2grid(:,:,k+1)* inv(Hcam2grid(:,:,k)) ; % relative poses for camera
      Bind(:,:,k)=(Hcam2grid(:,:,k)); 
      WptsC(:,:,k)=Bind(:,:,k)* Wpts; %3D Grid pts in Cam frame for each pose
  end

   


%% using optimizer

    options = optimoptions(@fmincon,'Algorithm','interior-point','Display','off');
    cF1 = @(est) costFunc1(imgPts,WptsC,K,A, est);
    [x1,~] = fmincon(cF1,initial,[],[],[],[],[],[],[],options);

 

    XR1out=quat2rotm(x1(1:4));%base to world grid
    Xt1out=x1(5:7);
    Xc1=createHT(XR1out,Xt1out); %HT base to grid
    time=toc;
%% defining cost functions

function [err] = costFunc1(imgPts,WptsC,K,Aarr,x)
    p=1;
    selectRow12=@(x) x(1:2,:);
    normbyZ = @(ptProj) selectRow12(bsxfun(@rdivide, ptProj, ptProj(3,:)));% % Divide with Z to complete the perspective projection and the removing the unit 1  as a resultant of the division
err=zeros(size(imgPts,1),1);
    for t=1:length(Aarr)
              W=WptsC(:,:,t);
              A=Aarr(:,:,t);
              imgP=imgPts(:,:,t+1);%for projecting the points on the image acquired  through next pose


%                comb = @(val) norm(val)/sqrt(length(val)); 
               HT = @(val)   (   inv([[quat2rotm(val(1:4)) val(5:7)']; [0 0 0 1] ]) * A *  ([[quat2rotm(val(1:4)) val(5:7)']; [0 0 0 1] ])  )  ; % theoritical relation inv(X)*inv(A)*X  
               proj=(  normbyZ(K*((HT(x)))*W) )'; 
               err(:,t) = sum((imgP - proj).^2,2);


    end



err=mean(sqrt(err(:)));  % fin the mean of the magnitude  mean (sqrt(x.^2+y.^2))


