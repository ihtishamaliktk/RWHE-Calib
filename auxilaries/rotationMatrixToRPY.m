% Convert a 3x3 matrix to RPY
% homMatrix: 3x3 matrix

function [ roll, pitch, yaw ] = rotationMatrixToRPY( homMatrix )
%roll, pitch, yaw in degrees
quat = rotm2quat(homMatrix(1:3, 1:3));
[yaw, pitch, roll] = quat2angle(quat);

yaw = rad2deg(yaw);
pitch = rad2deg(pitch);
roll = rad2deg(roll);

end

