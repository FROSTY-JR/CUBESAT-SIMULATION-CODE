%% ////////////////////////////////////////////////////////////////////////
%  EULER ANGLES TO QUATERNION HELPER FUNCTION
%  ////////////////////////////////////////////////////////////////////////
%  Description:
%  Converts a set of ZYX Euler angles (Yaw, Pitch, Roll) into a
%  rotation quaternion.
%
%  Inputs: yaw, pitch, roll - angles in radians.
%  Output: q - 4x1 column vector of the corresponding quaternion.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = euler_to_quaternion(yaw, pitch, roll)
    % Calculate half angles
    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);
    cp = cos(pitch * 0.5);
    sp = sin(pitch * 0.5);
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);

    % ZYX sequence
    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
    
    q = [qw; qx; qy; qz];
end