% The function is dependent on the  MATLABS Camera Calibration toolkit
% IComputes intrinsic calibration based on input images
% calibImgPaths: Pathvector to input images
% square: Checkerboard square size in mm size

function [cameraParams, UndistoredImgNPts,OrigImgNPts,imagesUsed, correctedPoses, estimationErrors,worldPoints] = CalibrateIntrinsics(calibImgPath, squareSize,Config)

% Get Image names
fileList = [dir(fullfile(calibImgPath,'*png')); dir(fullfile(calibImgPath,'*jpg'));dir(fullfile(calibImgPath,'*bmp')); dir(fullfile(calibImgPath,'*tiff'))];
calibImgNames = extractfield(fileList,'name');
calibImgFullname=strcat(calibImgPath,calibImgNames);


% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(calibImgFullname,'MinCornerMetric',0.12, 'showProgressBar', 1);
calibImgNames = calibImgNames(imagesUsed); %checken!!!
imagePointsO=imagePoints;
% Generate world coordinates of the corners of the squares
% in units
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Load one image to get size
img = imread([calibImgPath,'\',calibImgNames{1}]);
imgSize = size(img);

% Calibrate the camera
[cameraParams, imagesUsedErrors, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', true, 'EstimateTangentialDistortion', true, ...
    'NumRadialDistortionCoefficients', 3, 'WorldUnits', 'm', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], 'ImageSize', imgSize(1:2));

    % Remove all images depending on errors
    j = 1;
    for i = 1:length(imagesUsed)
        if (imagesUsed(i))
            if (imagesUsedErrors(j) == 0)
                imagesUsed(i) = 0;
            end
            j = j + 1;
        end
    end
    
   
    total = size(imagesUsedErrors, 1);
    h = waitbar(0, ['Computing extrinsics for image: 1 of ' num2str(total)],...
                    'Name', 'Undistorting image origins');
    % Undistort image origins                
    for i = 1:total
        waitbar(i/total, h, ['Computing extrinsics for image: ' num2str(i) ' of ' num2str(total)]);
        if imagesUsedErrors(i)
            orgImg = imread([calibImgPath,'\',calibImgNames{i}]);
            
            
%             [img, newOrigin] = undistortImage(orgImg, cameraParams, 'OutputView', 'full');
            [img, newOrigin] = undistortImage(orgImg, cameraParams);

            [imagePoints(:,:,i), ~] = detectCheckerboardPoints(img ,'MinCornerMetric',0.12);
            UndistoredImgNPts{i,1}=img; % storing the undistored images
            UndistoredImgNPts{i,2}=imagePoints(:,:,i); % storing the corner pts of undistored images
            
            % Induce Noise
            if ~isempty(Config.Np)
                [r,c]=size(UndistoredImgNPts{1,2});
                mu=0;sigma=Config.Np(1);
                  VisNoise=mu+randn(r,c).*sigma; % normally distributed with random spikes possible
%                 VisNoise=-sigma + (sigma+sigma).*rand(r,c); % uniformly distributed with in limits
                UndistoredImgNPts{i,2}=UndistoredImgNPts{i,2}+VisNoise;
            end
            
%             imagePoints(:,:,i) = [imagePoints(:,1, i) + newOrigin(1),imagePoints(:,2,i) + newOrigin(2)]; % when this new Origin is added back, it goe back to orignal image cordinate, This adjustment
              imagePoints(:,:,i)=imagePointsO(:,:,i);
              
            % makes it compatible with the cameraParameters object computed for the original image.
             OrigImgNPts{i,1}=orgImg; % storing the original images
             OrigImgNPts{i,2}=imagePoints(:,:,i); % storing the corner pts of original images
           
             
               % Save new calibration data
%             [correctedOrientationMatrix, correctedTranslationVector] = extrinsics(...
%                 imagePoints(:,:,i), worldPoints, cameraParams); % transformations from the grid/world coordinates
                        [correctedOrientationMatrix, correctedTranslationVector] = extrinsics(...
                UndistoredImgNPts{i,2}, worldPoints, cameraParams); % transformations from the grid/world coordinates
%                       to the camera-based coordinates
            correctedPoses.correctedOrientationMatrix(:,:,i) = correctedOrientationMatrix;
            correctedPoses.correctedTranslationVector(:,:,i) = correctedTranslationVector;
           
        end
    end
    delete(h);
end

