function R = fn_RFromAngVec( angVec )
alpha = angVec(1);
beta = angVec(2);
gamma = angVec(3);
R = fn_Rx(alpha) * fn_Ry(beta) * fn_Rz(gamma);
