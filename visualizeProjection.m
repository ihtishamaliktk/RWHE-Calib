function []=visualizeProjection(B2W,H2E,imgPts,Worldpts,Aarr,ImgCell,K,B,formulation)%% Definitions
%

%% Definitions and initialization
selectRow12=@(x) x(1:2,:);
normbyZ = @(ptProj) selectRow12(bsxfun(@rdivide, ptProj, ptProj(3,:)));% % Divide with Z to complete the perspective projection and the removing the unit 1  as a resultant of the division
p=1;
Wpts = [Worldpts, zeros(size(Worldpts,1),1), ones(size(Worldpts,1),1)]';
% w2cv=[1 0 0 0;0 -1 0 0;0 0 -1 0;0 0 0 1];% for blender synthetic data

disp('press any key to view next frame...')


 if strcmp(formulation,'AXZB')
    %%changing format
    x =[ rotm2quat(B2W(1:3,1:3)) B2W(1:3,4)'  rotm2quat(H2E(1:3,1:3)) ,H2E(1:3,4)'];
    for m=1:length(Aarr)
    % m=1;
        A=Aarr(:,:,m);
        img=ImgCell{m};
        imgP=imgPts(:,:,m);
        %projecting to image plane 
         HT = @(val)   (   inv([[quat2rotm(val(8:11)) val(12:14)']; [0 0 0 1] ]) * A *  ([[quat2rotm(val(1:4)) val(5:7)']; [0 0 0 1] ])  )  ; % either inv(Z)*A*X  or Z*A*X
         invR=@(val)      createHT((val(1:3,1:3))',val(1:3,4));
         proj{m}=(  normbyZ(K*((HT(x)))*Wpts) )'; %need to invert HT befor invR, if the representation change of whole inverse is followed
        %  err= ([double(imgP) - proj]     ).^p;
        % 
%          HT = B(:,:,m);
%          proj{m}=(  normbyZ(K*((HT))*Wpts) )'; %need to invert HT befor invR, if the representation change of whole inverse is followed
%          err= ([double(imgP) - proj]     ).^p;

         figure(101),imshow(img)
         hold on
         scatter(imgP(:,1), imgP(:,2),[],'g','o')
         hold on
         pt=proj{m};
         scatter(pt(:,1), pt(:,2),[],'r', 'x')
         w = waitforbuttonpress;


    end


 elseif strcmp(formulation,'AXXB')
     x =[rotm2quat(H2E(1:3,1:3)) ,H2E(1:3,4)'];

     n = size(imgPts,3);
     for k=1:n-1
          Arel(:,:,k)= inv(Aarr(:,:,k+1))* Aarr(:,:,k); % relative poses for Gripper
          Bind(:,:,k)=(B(:,:,k)); 
          WptsC(:,:,k)=Bind(:,:,k)* Wpts; %3D Grid pts in Cam frame for each pose
     end

    for m=1:length(Arel)
         At=Arel(:,:,m);
         img=ImgCell{m+1};
         imgP=imgPts(:,:,m+1);
         %projecting to image plane 
         HT = @(val)   (   inv([[quat2rotm(val(1:4)) val(5:7)']; [0 0 0 1] ]) * At *  ([[quat2rotm(val(1:4)) val(5:7)']; [0 0 0 1] ])  )  ; %  inv(X)*inv(A)*X  
         proj{m}=(  normbyZ(K*((HT(x)))*WptsC(:,:,m)) )';


         figure(101),imshow(img)
         hold on
         scatter(imgP(:,1), imgP(:,2),[],'g','o')
         hold on
         pt=proj{m};
         scatter(pt(:,1), pt(:,2),[],'r', 'x')
         w = waitforbuttonpress;

    end
     
 end
 disp('All images viewed.')
 %% visualize
