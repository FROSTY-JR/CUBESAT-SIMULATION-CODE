%% ////////////////////////////////////////////////////////////////////////
%  QUATERNION MULTIPLICATION HELPER FUNCTION (Fully Corrected)
%  ////////////////////////////////////////////////////////////////////////

function q_product = quat_multiply(q1, q2)
    qw1 = q1(1);
    qx1 = q1(2);
    qy1 = q1(3);
    qz1 = q1(4);
    
    qw2 = q2(1);
    qx2 = q2(2);
    qy2 = q2(3);
    qz2 = q2(4);
    
    % Correct Hamilton Product Implementation
    qw_new = qw1*qw2 - qx1*qx2 - qy1*qy2 - qz1*qz2;
    qx_new = qw1*qx2 + qx1*qw2 + qy1*qz2 - qz1*qy2;
    qy_new = qw1*qy2 - qx1*qz2 + qy1*qw2 + qz1*qx2;
    qz_new = qw1*qz2 + qx1*qy2 - qy1*qx2 + qz1*qw2;
    
    q_product = [qw_new; qx_new; qy_new; qz_new];
end