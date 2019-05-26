% Create a 3x3 matrix from RPY
% angleRoll: Roll in deg
% anglePitch: Pitch in deg
% angleYaw: Yaw in deg

function matrix = rpyToRotationMatrix( angleRoll, anglePitch, angleYaw )
rz = [cosd(angleYaw) -sind(angleYaw) 0  ;
    sind(angleYaw) cosd(angleYaw) 0;
    0 0 1];

ry = [ cosd(anglePitch) 0 sind(anglePitch);
    0 1 0;
    -sind(anglePitch) 0 cosd(anglePitch)
    ];

rx = [ 1 0 0;
    0 cosd(angleRoll) -sind(angleRoll);
    0 sind(angleRoll) cosd(angleRoll)];

matrix = rz*ry*rx;

end

