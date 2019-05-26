function outPosesHT=inducePoseNoise(posesHT,Config)
% Induces Rotational and Translation noise into the poses 

  n=size(posesHT,3);
  % rotational noise
    if ~isempty(Config.Nr)
        meuRy=Config.Nr(1,1);sigmaRy=Config.Nr(1,2);
        meuRp=Config.Nr(2,1);sigmaRp=Config.Nr(2,2);
        meuRr=Config.Nr(3,1);sigmaRr=Config.Nr(3,2);
%         RotNoise=-sigmaR + (sigmaR+sigmaR).*rand(n,3); % uniformly distributed with in limit4 
      RotNoise=meuRy+randn(n,1).*sigmaRy; % normally distributed with random spikes possible
      RotNoise(:,2)=meuRp+randn(n,1).*sigmaRp; % normally distributed with random spikes possible
      RotNoise(:,3)=meuRr+randn(n,1).*sigmaRr; % normally distributed with random spikes possible

    else
        RotNoise=zeros(n,3);
    end
  %translation noise
    if ~isempty(Config.Nt)
%         muT=0.00031;sigmaT=Config.Nt(1);
        meuX=Config.Nt(1,1);sigmaX=Config.Nt(1,2);
        meuY=Config.Nt(2,1);sigmaY=Config.Nt(2,2);
        meuZ=Config.Nt(3,1);sigmaZ=Config.Nt(3,2);
        TransNoise=meuX+randn(n,1).*sigmaX; % normally distributed with random spikes possible
        TransNoise(:,2)=meuY+randn(n,1).*sigmaY; % normally distributed with random spikes possible
        TransNoise(:,3)=meuZ+randn(n,1).*sigmaZ; % normally distributed with random spikes possible

    else
        TransNoise=zeros(n,3);
    end
      
   %adding to poses 
    for k=1:n
       NoiseHT=createHT(eul2rotm([RotNoise(k,1) RotNoise(k,2) RotNoise(k,3)]*(pi/180)),TransNoise(k,:)); %input order Yaw Pitch Roll , ''ZYX'
       outPosesHT(:,:,k)=NoiseHT*posesHT(:,:,k);
    end

