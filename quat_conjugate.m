%% ////////////////////////////////////////////////////////////////////////
%  QUATERNION CONJUGATE HELPER FUNCTION
%  ////////////////////////////////////////////////////////////////////////
%  Description:
%  Calculates the conjugate of a quaternion.
%
%  Input: q - 4x1 column vector representing the quaternion.
%  Output: q_conj - 4x1 column vector of the conjugate.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q_conj = quat_conjugate(q)
    q_conj = [q(1); -q(2); -q(3); -q(4)];
end