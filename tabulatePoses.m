function tabulatePoses(inputStruct)

%the function takes all the homogenous transformations converts them to
%eular angles and prints in a tabular form for easy understanding

%% Print Results
%re-arranging data
 Carr = struct2cell(inputStruct);
    for k=1:length(Carr)
     content=Carr{k};
%      ypr=(rotm2eul(content(1:3,1:3)))*(180/pi);
%      yaw=ypr(1);pitch=ypr(2);roll=ypr(3);
         if any(isnan(content(1:3,1:3)))
           roll=nan;pitch=nan;yaw=nan;t=[nan nan nan];
         else
             [ roll, pitch, yaw ] = rotationMatrixToRPY( content(1:3,1:3) );
             t=content(1:3,4);
         end
         
      xyzrpy.x(k,1)=t(1); xyzrpy.y(k,1)=t(2); xyzrpy.z(k,1)=t(3);
      xyzrpy.R(k,1)=roll; xyzrpy.P(k,1)=pitch; xyzrpy.Y(k,1)=yaw;
     
    end

ResultsTT = table(xyzrpy.x,xyzrpy.y,xyzrpy.z,xyzrpy.R,xyzrpy.P,xyzrpy.Y);
ResultsTT.Properties.VariableNames = {'x_m','y_m','z_m','roll_deg','pitch_deg','yaw_deg'};
ResultsTT.Properties.RowNames  = fieldnames(inputStruct);

ResultsTT

end
