function [formulations,errors]= computeErrors(Hbase2hand,Hhand2eye,Hbase2grid,Hcam2grid,GT,imgPts,Worldpts,K)


%where the Input:
%Hbase2hand         a 4x4xNumber_of_Views Matrix of the form
%                   Hbase2hand(:,:,i) = [Ri_3x3 ti_3x1;[ 0 0 0 1]] 
%                   with 
%                   i = number of the view, 
%                   Ri_3x3 the rotation matrix 
%                   ti_3x1 the translation vector.
%                   Defining the transformation from the robot base to gripper / marker
%Hhand2eye          a 4x4xNumber_entries_to_examine Matrix of the form
%                   Hhand2eye(:,:,i) = [Ri_3x3 ti_3x1;[ 0 0 0 1]] 
%Hbase2grid         a 4x4xNumber_entries_to_examine Matrix of the form
%                   Hbase2grid(:,:,i) = [Ri_3x3 ti_3x1;[ 0 0 0 1]]    
%Hcam2grid          a 4x4xNumber_of_Views Matrix (like above)
%                   Defining the transformation of the  camera to the
%                   grid/calibration block/world
%K                  The camera intrinsic  (4x4 matrix)
%imgPts             a cell array with each cell containing the imgpoints corresponding to the corner points on the
%                   calibration board.(Nx2xM), where N is number of corners
%                   and M is the number of calibration images
%Worldpts           world coordinates of the corners (Nx2) matrix
%GT                 A 4x4 matrix with the Ground truth Hhand2eye

% The function checks the presence of a variable a defines the problem formulation based on the inputs provided e.g
% if inputs are [Hbase2gripper(:,:,3),Hhand2eye(:,:,3),[],Hcam2grid(:,:,3),GT(:,:,3)] then it is assumed that the problem is of AXXB nature and a Ground Truth Pose is present for this problem

%Output:
%Rerr               The rotation error in degrees corresponding to each
%                   each entry
%terr               The translation error in meters corresponding to each each entry
%Rterr              The combined Rotation and translation error corresponding to each each entry
%ReprojErr          The reprojection error from reprojecting the world/grid
%                   points on the camera images corresponding to each each entry
%AbsoluteErr        The combined error from ground truth Transformations
%                   corresponding to each entry


%NOTE: Since two formulation AXXB and AXZB are handles ,thus important
%variables will be followed by by X and Z  for AXXB and AXZB respectively
%such as  A_X,A_Z,B_X,B_Z,X_X,X_Z,Z_Z

n=size(imgPts,3);
%% computing dependencies
selectRow12=@(x) x(1:2,:);
normbyZ = @(ptProj) selectRow12(bsxfun(@rdivide, ptProj, ptProj(3,:)));% % Divide with Z to complete the perspective projection 
Wpts = [Worldpts, zeros(size(Worldpts,1),1), ones(size(Worldpts,1),1)]';
for p=1:n
Hhand2base(:,:,p)=inv(Hbase2hand(:,:,p)); % for AXZB formulations
end
for p=1:n-1
    Arel_X(:,:,p)= inv(Hbase2hand(:,:,p+1))* Hbase2hand(:,:,p); % relative poses for Gripper for AXXB formulation
    Brel_X(:,:,p)= Hcam2grid(:,:,p+1)* inv(Hcam2grid(:,:,p)) ; % relative poses for camerafor AXXB formulation
    WptsC_X(:,:,p)=Hcam2grid(:,:,p)* Wpts; %3D Grid pts in Cam frame for each pose for AXXB formulation
end



%% Computing Errors
for i=1:size(Hhand2eye,3)
 if isnan(Hbase2grid(1,1,i))
    formulation='AXXB';
 elseif all(size(Hbase2grid(:,:,i))==[4 4])
    formulation='AXZB';
 else
     error('Hbase2grid is not a 4x4 Homogenous Transformation')
 end
 formulations{i,1}=formulation;
 
    
%% Computing Rotation errors

% For AXXB
if strcmp(formulation,'AXXB')
    angR=[];PoseErrR=[];
    for j=1:length(Arel_X)
        PoseErrR(:,:,j)= inv(Hhand2eye(1:3,1:3,i)*Brel_X(1:3,1:3,j))*(Arel_X(1:3,1:3,j)*Hhand2eye(1:3,1:3,i)) ;         % inv(Rx*Rb)(Ra*Rx)
%         [roll, pitch, yaw ] = rotationMatrixToRPY(PoseErrR(:,:,j));%output in degrees
%         angR(j,:)= [ roll  pitch yaw ];
        angR(j,:) = rotm2eul(PoseErrR(:,:,j),'XYZ')*(180/pi);
     
    end

    Rerr(i,1)= mean(sqrt(sum(angR.^2,2))); 


% For AXZB
elseif strcmp(formulation,'AXZB')
    angR=[];PoseErrR=[];
    for j=1:length(Hcam2grid)
     PoseErrR(:,:,j)= inv(Hhand2eye(1:3,1:3,i)*Hcam2grid(1:3,1:3,j))*(Hhand2base(1:3,1:3,j)*Hbase2grid(1:3,1:3,i)) ;         % inv(Rz*Rb)(Ra*Rx)
%      [ roll, pitch, yaw ] = rotationMatrixToRPY(PoseErrR(:,:,j));%output in degrees
%       angR(j,:)= [ roll  pitch yaw ];
        angR(j,:) = rotm2eul(PoseErrR(:,:,j),'XYZ')*(180/pi);
    end

    Rerr(i,1)= mean(sqrt(sum(angR.^2,2))); 

end
%% Computing Translation  errors

% For AXXB
if strcmp(formulation,'AXXB')
    PoseErrt=[];
    for j=1:length(Arel_X)
    PoseErrt(j,:)= ( ((Arel_X(1:3,1:3,j)*Hhand2eye(1:3,4,i))+ Arel_X(1:3,4,j)) -...
                    ((Hhand2eye(1:3,1:3,i)*Brel_X(1:3,4,j))+Hhand2eye(1:3,4,i) ) )'  ;%(Rar*tx)+tar -(Rx*tbr)+tx
   
    end
    terr(i,1)=mean(sqrt(sum(PoseErrt.^2,2))); 

% For AXZB
elseif strcmp(formulation,'AXZB')
    PoseErrt=[];
    for j=1:length(Hcam2grid)
    PoseErrt(j,:)= ( ((Hhand2base(1:3,1:3,j)*Hbase2grid(1:3,4,i))+ Hhand2base(1:3,4,j)) -...
                    ((Hhand2eye(1:3,1:3,i)*Hcam2grid(1:3,4,j))+Hhand2eye(1:3,4,i) ) )'  ;%(Ra*tx)+ta -(Rz*tb)+tz
   
    end
    terr(i,1)=mean(sqrt(sum(PoseErrt.^2,2))); 

end

%% Computing Reprojection  errors

% For AXXB
if strcmp(formulation,'AXXB')
    err=[];
        for j=1:length(Arel_X)
          imgP=imgPts(:,:,j+1);%for projecting the points on the image acquired  through next pose
          HT =  inv(Hhand2eye(:,:,i)) * Arel_X(:,:,j) *  Hhand2eye(:,:,i)  ;  
          proj=  normbyZ(K*(HT*WptsC_X(:,:,j)) )'; %need to invert HT befor invR, if the representation change of whole inverse is followed                        
%             err(:,j) = sqrt(sum((imgP - proj).^2,2)); 
          err(:,j) = sum((imgP - proj).^2,2);%squared error

        end
        errreproj(i,:)=sqrt(mean(err(:)));%rrmse

% For AXZB
elseif strcmp(formulation,'AXZB')
    err=[];
        for j=1:length(Hhand2base)
          A=Hhand2base(:,:,j);
          imgP=imgPts(:,:,j);
          HT =    inv(Hhand2eye(:,:,i)) * A *  Hbase2grid(:,:,i)    ; % either inv(Z)*A*X  ,where Z is H2e and X is base 2 grid
          proj=  normbyZ(K*(HT*Wpts) )'; %need to invert HT befor invR, if the representation change of whole inverse is followed                        
%           err(j,1) = sqrt(sum((imgP - proj).^2,2)); %generic
          err(:,j) = sum((imgP - proj).^2,2);% adopted approach

        end
        errreproj(i,:)=sqrt(mean(err(:)));%rrmse

end

%% Computing absolute Rotation  error from ground truth
    if ~isempty(GT)
         diffR=inv(Hhand2eye(1:3,1:3,i))*GT(1:3,1:3);
%          [ roll, pitch, yaw ] = rotationMatrixToRPY( diffR )  ;      % diffR
%           angabsR= [ roll  pitch yaw ];

         XYZ = rotm2eul(diffR(1:3,1:3),'XYZ')*(180/pi);
         angabsR= XYZ;
         AbsRerr(i,:)= mean(sqrt(sum(angabsR.^2,2)));  
     else
        AbsRerr(i,:)=nan;
    end



%% Computing absolute translation  error from ground truth
    if ~isempty(GT)
         difft=GT(1:3,4)-Hhand2eye(1:3,4,i);
         Absterr(i,:)= mean(sqrt(sum(difft.^2,2)));
    else
        Absterr(i,:)=nan;
    end

end

errors=[Rerr terr errreproj AbsRerr Absterr];