function Configuration=H2EConfiguration
%% Methods: Set methods true using which to compute Hand eye Calibration


%Solver Algorithm type
Configuration.SolAlgo='interior_point';
% Configuration.SolAlgo='levenberg_marquardt';


% Compute Error

%induce noise  mean and sigma values for x y z
% If no noise is intended, then set all the noise parameters to []
% Configuration.Np=[];% noise added in pixels  0.25 or 0.5
% Configuration.Nt=1*[0.00006  0.00022; 
%                  -0.00005  0.00018;
%                  -0.00004  0.00017];% noise added to translation component of robot poses in meters
% Configuration.Nr=1*[0.0032   0.0177;
%                  -0.0002   0.0161;
%                   0.0002   0.0110];% noise added to rotational component of robot poses in degrees
%       

Configuration.Np=[];% noise added in pixels  0.25 or 0.5
Configuration.Nt=[];% noise added to translation component of robot poses in meters
Configuration.Nr=[];% noise added to rotational component of robot poses in degrees
              