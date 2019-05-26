  function poses = createPosesFromCameraParams(cameraParams)
            poses.correctedOrientationMatrix = cameraParams.RotationMatrices;
            
            for i = 1:size(cameraParams.TranslationVectors, 1)
                poses.correctedTranslationVector(1, :, i) = cameraParams.TranslationVectors(i, :);
            end
            
  end