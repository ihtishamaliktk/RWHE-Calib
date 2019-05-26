function [Xc1,Zc1,Xc2,Zc2,time1,time2] = ProposedAXZB(Hgripper2base,Hcam2grid,initial,Configuration) 
%where the Input:
%Hbase2gripper      a 4x4xNumber_of_Views Matrix of the form
%                   Hgripper2base(:,:,i) = [Ri_3x2 ti_3x1;[ 0 0 0 1]] 
%                   with 
%                   i = number of the view, 
%                   Ri_3x3 the rotation matrix 
%                   ti_3x1 the translation vector.
%                   Defining the transformation from the  gripper / marker to robot base
%Hcam2grid          a 4x4xNumber_of_Views Matrix (like above)
%                   Defining the transformation of the  camerato the
%                   grid/calibration block/world
%
%Output:
%Zc                 The transformation from gripper/marker to the camera  for the
%                   specific costfunction
%Xc                 The transformation from base to calibrtion grid/world  for the
%                   specific costfunction


  n = size(Hgripper2base,3);
  A=Hgripper2base;
  B=Hcam2grid;



%% using optimizer
% options.Algorithm = 'levenberg-marquardt';
% options.ScaleProblem='jacobian';
% options.Display='off';
% 
% tic
% %computing for cF1
% cF1 = @(est) costFunc1(A, B, est);
% [x1,resnorm,residual,exitflag,output] = lsqnonlin(cF1,initial,[],[],options);
% XR1out=quat2rotm(x1(1:4));%base to world grid
% Xt1out=x1(5:7);%base to world grid
% ZR1out=quat2rotm(x1(8:11));%gripper to cam
% Zt1out=x1(12:14);%gripper to cam
% Xc1=createHT(XR1out,Xt1out); %HT base to grid
% Zc1=createHT(ZR1out,Zt1out);%HT gripper to cam
% time1=toc;
% 
% %computing for cF2
% tic
% cF2 = @(est) costFunc2(A, B, est);
% [x2,resnorm,residual,exitflag,output] = lsqnonlin(cF2,initial,[],[],options);
% XR2out=quat2rotm(x2(1:4));%base to world grid
% Xt2out=x2(5:7);%base to world grid
% ZR2out=quat2rotm(x2(8:11));%gripper to cam
% Zt2out=x2(12:14);%gripper to cam
% Xc2=createHT(XR2out,Xt2out); %HT base to grid
% Zc2=createHT(ZR2out,Zt2out);%HT gripper to cam
% time2=toc;





%% using optimizer for cost 1
tic
 if strcmp(Configuration.SolAlgo,'levenberg_marquardt')
    options.Algorithm = 'levenberg-marquardt';
    options.ScaleProblem='jacobian';
    options.Display='off';
 
    % %computing for cF1
    cF1 = @(est) costFunc1(A, B, est);
    [x1,resnorm1,residual1,exitflag1,output1] = lsqnonlin(cF1,initial,[],[],options);

elseif strcmp(Configuration.SolAlgo,'interior_point')
    % using fmin
    options = optimoptions(@fmincon,'Algorithm','interior-point','Display','off');
    cF1 = @(est) costFunc1(A, B, est);
    [x1,~] = fmincon(cF1,initial,[],[],[],[],[],[],[],options);

 end

    XR1out=quat2rotm(x1(1:4));%base to world grid
    Xt1out=x1(5:7);%base to world grid
    ZR1out=quat2rotm(x1(8:11));%gripper to cam
    Zt1out=x1(12:14);%gripper to cam
    Xc1=createHT(XR1out,Xt1out); %HT base to grid
    Zc1=createHT(ZR1out,Zt1out);%HT gripper to cam
   time1=toc;
 
    
    
    
    
    %% using optimizer for cost 2
    tic
 if strcmp(Configuration.SolAlgo,'levenberg_marquardt')
    options.Algorithm = 'levenberg-marquardt';
    options.ScaleProblem='jacobian';
    options.Display='off';
 
    % %computing for cF2
    cF2 = @(est) costFunc2(A, B, est);
    [x2,resnorm2,residual2,exitflag2,output2] = lsqnonlin(cF2,initial,[],[],options);

elseif strcmp(Configuration.SolAlgo,'interior_point')
    % using fmin
    options = optimoptions(@fmincon,'Algorithm','interior-point','Display','off');
    cF2 = @(est) costFunc2(A, B, est);
    [x2,~] = fmincon(cF2,initial,[],[],[],[],[],[],[],options);

 end

    XR2out=quat2rotm(x2(1:4));%base to world grid
    Xt2out=x2(5:7);%base to world grid
    ZR2out=quat2rotm(x2(8:11));%gripper to cam
    Zt2out=x2(12:14);%gripper to cam
    Xc2=createHT(XR2out,Xt2out); %HT base to grid
    Zc2=createHT(ZR2out,Zt2out);%HT gripper to cam
time2=toc;

















%% defining cost functions

function [err] = costFunc1(Aarr,Barr,x)
for t=1:length(Aarr)
A=Aarr(:,:,t); B=Barr(:,:,t);

                      comb = @(val) sum([quatnorm(val(1:4)) norm(val(5:7))]);
                      transform = @(val) [rotm2quat(val(1:3,1:3)) val(1:3,4)'];
                      relation= [A*[[quat2rotm([x(1) x(2) x(3) x(4)]) x(5:7)']; [0 0 0 1] ]]  -  [ [[quat2rotm([x(8) x(9) x(10) x(11)]) x(12:14)']; [0 0 0 1] ]* B] ;%AX-ZB
                      err(t,:)= comb(transform(relation));

end

function [err] = costFunc2(Aarr,Barr,x)
for t=1:length(Aarr)
A=Aarr(:,:,t); B=Barr(:,:,t);

                    comb = @(val) sum([quatnorm(val(1:4)) norm(val(5:7))]);
                    transform = @(val) [rotm2quat(val(1:3,1:3)) val(1:3,4)'];
                    relation= [A-([[quat2rotm([x(8) x(9) x(10) x(11)]) x(12:14)']; [0 0 0 1] ]*B * inv([[quat2rotm([x(1) x(2) x(3) x(4)]) x(5:7)']; [0 0 0 1] ]))] ; %A-  ZBinv(X)
                    err(t,:)= comb(transform(relation));

end
  