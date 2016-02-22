%% CovInv object to matrix
function CovInv_mat = Cov_Object2Matrix( Cov_obj, nRow )

    nUv = size( Cov_obj.fObs.val, 1 );
    nIntlDelta = size( Cov_obj.intlDelta.val, 1 );
    nAu2c = size( Cov_obj.Au2c.val, 1 );
    nTu2c = size( Cov_obj.Tu2c.val, 1 );
    nBf = size( Cov_obj.Bf.val, 1 );
    nBw = size( Cov_obj.Bw.val, 1 );

    nCol = nUv + nIntlDelta + nAu2c + nTu2c + nBf + nBw;
    assert( nRow == nCol, 'Fatal: Covraince column count not same as Z observation row count' );


    %% init
    CovInv_mat = eye( nCol );                
    rCnt = 0;

    %% fill in UV observations
    CovInv_mat( rCnt + 1 : rCnt + nUv, rCnt + 1 : rCnt + nUv ) = Cov_obj.fObs.val;
    rCnt = rCnt + nUv;

    %% fill in inertial delta observation
    CovInv_mat( rCnt + 1 : rCnt + nIntlDelta, rCnt + 1 : rCnt + nIntlDelta ) = Cov_obj.intlDelta.val;
    rCnt = rCnt + nIntlDelta;

    %% fill in Au2c
    CovInv_mat( rCnt + 1 : rCnt + nAu2c, rCnt + 1 : rCnt + nAu2c ) = Cov_obj.Au2c.val;
    rCnt = rCnt + nAu2c;

    %% fill in Tu2c
    CovInv_mat( rCnt + 1 : rCnt + nTu2c, rCnt + 1 : rCnt + nTu2c ) = Cov_obj.Tu2c.val;
    rCnt = rCnt + nTu2c;

    %% fill in Bf
    CovInv_mat( rCnt + 1 : rCnt + nBf, rCnt + 1 : rCnt + nBf ) = Cov_obj.Bf.val;
    rCnt = rCnt + nBf;

    %% fill in Bw
    CovInv_mat( rCnt + 1 : rCnt + nBw, rCnt + 1 : rCnt + nBw ) = Cov_obj.Bw.val;
    rCnt = rCnt + nBw;                
return

    