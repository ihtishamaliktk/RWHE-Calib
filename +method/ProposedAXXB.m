function [Hc1,Hc2,time1,time2] = ProposedAXXB(Hbase2gripper,Hcam2grid,initial) 
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
%
%Output:
%Hc                 The transformation from gripper/marker to the camera  for the
%                   specific costfunction


tic
  A = [];
  B=  [];
  n = size(Hbase2gripper,3);
  
  for k=1:n-1
      A(:,:,k)= inv(Hbase2gripper(:,:,k+1))* Hbase2gripper(:,:,k); % relative poses for Gripper
      B(:,:,k)= Hcam2grid(:,:,k+1)* inv(Hcam2grid(:,:,k)) ; % relative poses for camera

  end
time=toc;

%% using optimizer
options.Algorithm = 'levenberg-marquardt';
options.ScaleProblem='jacobian';
options.Display='off';
% initial = [1,0,0,0,0,0,0];

tic
%computing for cF1
cF1 = @(est) costFunc1(A, B, est);
[x1,resnorm,residual,exitflag,output] = lsqnonlin(cF1,initial,[],[],options);
R1out=quat2rotm(x1(1:4));
t1out=x1(5:7);
Hc1=createHT(R1out,t1out);
time1=toc;
time1=time1+time;

%computing for cF2
tic
cF2 = @(est) costFunc2(A, B, est);
[x2,resnorm,residual,exitflag,output] = lsqnonlin(cF2,initial,[],[],options);
R2out=quat2rotm(x2(1:4));
t2out=x2(5:7);
Hc2=createHT(R2out,t2out);
time2=toc;
time2=time2+time;





%% defining cost functions

function [err] = costFunc1(Aarr,Barr,x)
for t=1:length(Aarr)
A=Aarr(:,:,t); B=Barr(:,:,t);

                      comb = @(val) sum([quatnorm(val(1:4)) norm(val(5:7))]);
                      transform = @(val) [rotm2quat(val(1:3,1:3)) val(1:3,4)'];%error convertor to euler to facilitate direct subtration
                      err(t,:)= comb(transform(     [A*[[quat2rotm([x(1) x(2) x(3) x(4)]) x(5:7)']; [0 0 0 1] ]]  -  [ [[quat2rotm([x(1) x(2) x(3) x(4)]) x(5:7)']; [0 0 0 1] ]* B]    ));
end


function [err] = costFunc2(Aarr,Barr,x)
for t=1:length(Aarr)
A=Aarr(:,:,t); B=Barr(:,:,t);

                    comb = @(val) sum([quatnorm(val(1:4)) norm(val(5:7))]);
                    transform = @(val) [rotm2quat(val(1:3,1:3)) val(1:3,4)'];%error convertor to euler to facilitate direct subtration
                    err(t,:)= comb(transform(     A-[[quat2rotm([x(1) x(2) x(3) x(4)]) x(5:7)']; [0 0 0 1] ] * B * inv( [[quat2rotm([x(1) x(2) x(3) x(4)]) x(5:7)']; [0 0 0 1] ])    ));

end
  
  
  
