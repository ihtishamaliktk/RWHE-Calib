function [tcpPoseInBaseCoords,gridPoseInCameraCoords,cameraPoseInGridCoords]=poserRepresentationConformability(RobotPoses,cameraPoses,validcalibposes,varargin)
 
% set the input pose nature e-g rpy,quaternion etc so that representation could be read and converted
% to homogenous tranformations
% Removes all poses based on reject images by intrinsic calibration
% RobotPoses: Cartesian calibration poses
% correctedPoeses: Corrected cartesian calibration poses
% validcalibposes: Mask (vector) with rejected and accepted images
%DataFormat: can be one of these where input can be 'rpy','quat','HT', where format should be  'xyz|rpy', 'xyz|quaternion','HT'
%%initialize
 persistent p;
    if isempty(p)
        % Set input parser
        defaults = struct('DataFormat', 'rpy');
          
        p = inputParser;
        p.CaseSensitive = false;

        p.addParameter('DataFormat', defaults.DataFormat); 
        parser = p;
    else
        parser = p;
    end
    parser.parse(varargin{:});
    DataFormat   = validatestring(parser.Results.DataFormat, {'rpy', 'quat','HTvector'}, mfilename, 'DataFormat');

    
    %% robot poses
% Set counter
j = 1;
for i = 1:size(RobotPoses, 1)
    if validcalibposes(i)
        
        % Convert to hom matrix
        if strcmp(DataFormat,'rpy') 
            pose = RobotPoses(j,:);       
            rotationMatrix = rpyToRotationMatrix(pose(4), pose(5), pose(6));%roll,pitch,yaw
            translationVec=[pose(1) pose(2) pose(3)];
            tform = createHT(rotationMatrix,translationVec);
            tcpPoseInBaseCoords(:,:,j) = tform; %normally
%             tcpPoseInBaseCoords(:,:,j) = invertHT(tform);
        
        elseif strcmp(DataFormat,'quat')
            error('crete the conversion function for quaterinion based o nthe input data, havent done this atm')
        
        elseif  strcmp(DataFormat,'HTvector')
             pose = RobotPoses(j,:);  
             tform= reshape(pose,4,[])'; 
             tcpPoseInBaseCoords(:,:,j) = tform;

        end
        
        j = j + 1;
    end
    
end

%%
% Set camera poses
j = 1;
for i = 1:size(validcalibposes)   
    if validcalibposes(i)
        % Convert to hom matrix
        gridPoseInCameraCoords_invR = inv(cameraPoses.correctedOrientationMatrix(:,:,j));%inverse of the just the orientation      
        gridPoseInCameraCoords(:,:,j)=createHT(gridPoseInCameraCoords_invR,cameraPoses.correctedTranslationVector(:,:,j));

        %deletehis    
        % [orientation,location] = extrinsicsToCameraPose(cameraPoses.correctedOrientationMatrix(:,:,j),cameraPoses.correctedTranslationVector(:,:,j)) % output is cam2 grid
        %  gridPoseInCameraCoords(:,:,j)=(createHT(orientation,location));

%         or - this should be the correct form, but not sure
%         gridPoseInCameraCoords(:,:,j)=inv(createHT(cameraPoses.correctedOrientationMatrix(:,:,j),cameraPoses.correctedTranslationVector(:,:,j)));


        cameraPoseInGridCoords(:,:,j) = inv(gridPoseInCameraCoords(:,:,j));%inverse of the orientation and translation
        
        j = j + 1;
    end
end


end

