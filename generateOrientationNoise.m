%% NOTE: This script require Robotics toolkit by peter corke
%% load robot model

% Load robot Puma 560 model
% mdl_puma560
% Robot=p560;

% Load robot Kuka KR5 model
mdl_KR5
Robot=KR5;

Robot.teach

%position error characterization in meters
n=100;%samples
meuX=0.00006;sigmaX=0.00022; % err in meters
meuY=-0.00005;sigmaY=0.00018;
meuZ=-0.00004;sigmaZ=0.00017;
meuJ=0.000572;sigmaJ=0.012;%error induced intentionally  per joint , works for p50,KR5 


dj=(meuJ+randn(n,6).*sigmaJ) *(pi/180); % normally distributed with random spikes possible

%specifiying position to reach in work space
P=[-0.452 -0.150 0.432];
P(2,:)=[0.472 -0.150 0.452];


Orient=eye(3);
Orient(:,:,2)=eye(3);
for positions=1:size(P,1)
T=createHT(Orient(:,:,positions),P(positions,:));    
% For various  configurations
    Jang1 = Robot.ikine6s(T, 'ru');
    Jang2 = Robot.ikine6s(T, 'rd');
    Jang3 = Robot.ikine6s(T, 'lu');
    Jang4 = Robot.ikine6s(T, 'ld');
    Jang5 = Robot.ikine6s(T, 'luf');
    Jang6 = Robot.ikine6s(T, 'ruf');
    Jang7 = Robot.ikine6s(T, 'rdf');%right, down ,wrist flipped


   % For each configuration angles , introduce noise and see if it remains inside he limit of telerance, and note orientations
    for iter=1:n
        Target1(:,:,iter)=Robot.fkine(Jang1+dj(iter,:));
        Target2(:,:,iter)=Robot.fkine(Jang2+dj(iter,:));
        Target3(:,:,iter)=Robot.fkine(Jang3+dj(iter,:));
        Target4(:,:,iter)=Robot.fkine(Jang4+dj(iter,:));
        Target5(:,:,iter)=Robot.fkine(Jang5+dj(iter,:));
        Target6(:,:,iter)=Robot.fkine(Jang6+dj(iter,:));
        Target7(:,:,iter)=Robot.fkine(Jang7+dj(iter,:));

    end
    
    % keeping only valid angles whose position remained within the limits
    % of position+error
     [Orient1,trans1,bool1]=checkRangeNextractHT(Target1,P(positions,:),[-sigmaX sigmaX],[-sigmaY sigmaY],[-sigmaZ sigmaZ]);    
     [Orient2,trans2,bool2]=checkRangeNextractHT(Target2,P(positions,:),[-sigmaX sigmaX],[-sigmaY sigmaY],[-sigmaZ sigmaZ]) ;   
     [Orient3,trans3,bool3]=checkRangeNextractHT(Target3,P(positions,:),[-sigmaX sigmaX],[-sigmaY sigmaY],[-sigmaZ sigmaZ]);  
     [Orient4,trans4,bool4]=checkRangeNextractHT(Target4,P(positions,:),[-sigmaX sigmaX],[-sigmaY sigmaY],[-sigmaZ sigmaZ]); 
     [Orient5,trans5,bool5]=checkRangeNextractHT(Target5,P(positions,:),[-sigmaX sigmaX],[-sigmaY sigmaY],[-sigmaZ sigmaZ]);
     [Orient6,trans6,bool6]=checkRangeNextractHT(Target6,P(positions,:),[-sigmaX sigmaX],[-sigmaY sigmaY],[-sigmaZ sigmaZ]);
     [Orient7,trans7,bool7]=checkRangeNextractHT(Target7,P(positions,:),[-sigmaX sigmaX],[-sigmaY sigmaY],[-sigmaZ sigmaZ]);
    if sum(bool7)==0
     error('Error induced too high for pose tolerence, or robot model is incorrect')
    else
    fprintf('valid poses inside error envelop : %d out of %d \n',sum(bool7),n)
    end

     %Keeping only the valid ones and finding the orientation and
     %position errors
     or1_err= computeErr(Orient1,bool1,T); % output is err in degrees through rotm2eul(,'ZYX')*(180/pi)
     or2_err= computeErr(Orient2,bool2,T);
     or3_err= computeErr(Orient3,bool3,T);
     or4_err= computeErr(Orient4,bool4,T);
     or5_err= computeErr(Orient5,bool5,T);
     or6_err= computeErr(Orient6,bool6,T);
     or7_err= computeErr(Orient7,bool7,T);

    allOrientErr_pos{positions,1}=[or1_err;or2_err;or3_err;or4_err;or5_err;or6_err;or7_err];
end
    meuR=mean(cell2mat(allOrientErr_pos));  sigmaR=std(cell2mat(allOrientErr_pos)); %Yaw pitch Roll in degrees
fprintf('meanR_ypr= %d \n',meuR);
fprintf('sigmaR_ypr= %d \n',sigmaR);


    
    
    
    
    
    
    
%=================defining functions===============
function [Orient,trans,boolout]=checkRangeNextractHT(HTs,refxyz,xtol,ytol,ztol)
% range :lower and then higher limis e-g [-5 5]
for k=1:size(HTs,3)
    HT=HTs(1,1,k); 
    xyz= HT.t;
    trans(k,:)=xyz;
    Orient(:,:,k)=t2r(HT);

    % check x
    boolx= xyz(1)>=refxyz(1)-abs(xtol(1)) & xyz(1)<=refxyz(1)+abs(xtol(2));
    booly= xyz(2)>=refxyz(2)-abs(ytol(1)) & xyz(2)<=refxyz(2)+abs(ytol(2));
    boolz= xyz(3)>=refxyz(3)-abs(ztol(1)) & xyz(3)<=refxyz(3)+abs(ztol(2));
    bool=and(boolx,booly);
    boolout(k,1)=and(bool,boolz);

end

end

function err= computeErr(Orientation,valid,GT)
    ind=find(valid);
    for k=1:length(ind)
        ValidInd=ind(k);
        diffR=inv(Orientation(:,:,ValidInd))*GT(1:3,1:3);
        err(k,:)=rotm2eul(diffR)*(180/pi);
    end
end

