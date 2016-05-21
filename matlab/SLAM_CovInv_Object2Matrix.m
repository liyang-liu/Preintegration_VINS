%% CovInv object to matrix
function CovInv_mat = SLAM_CovInv_Object2Matrix( CovInv_obj, nRow )

    global PreIntegration_options
    
    nUv = size( CovInv_obj.fObs.val, 1 );
    nIntlDelta = size( CovInv_obj.intlDelta.val, 1 );
    
    if ( PreIntegration_options.bAddZg == 1 )
        ng = size( CovInv_obj.g.val, 1 );
    else
        ng = 0;
    end
    nAu2c = size( CovInv_obj.Au2c.val, 1 );
    nTu2c = size( CovInv_obj.Tu2c.val, 1 );
    nBf = size( CovInv_obj.Bf.val, 1 );
    nBw = size( CovInv_obj.Bw.val, 1 );

    nCol = nUv + nIntlDelta + ng + nAu2c + nTu2c + nBf + nBw;
    assert( nRow == nCol, 'Fatal: Covraince column count not same as Z observation row count' );


    %% init
    CovInv_mat = eye( nCol );                
    rCnt = 0;

    %% fill in UV observations
    if ( nUv > 0 )
        CovInv_mat( rCnt + 1 : rCnt + nUv, rCnt + 1 : rCnt + nUv ) = CovInv_obj.fObs.val;
        rCnt = rCnt + nUv;
    end
    
    %% fill in inertial delta observation
    if ( nIntlDelta > 0 )
        CovInv_mat( rCnt + 1 : rCnt + nIntlDelta, rCnt + 1 : rCnt + nIntlDelta ) = CovInv_obj.intlDelta.val;
        rCnt = rCnt + nIntlDelta;
    end
    
    %% fill in gravity observation
    if ( ng > 0 )
        CovInv_mat( rCnt + 1 : rCnt + ng, rCnt + 1 : rCnt + ng ) = CovInv_obj.g.val;
        rCnt = rCnt + ng;
    end
    
    %% fill in Au2c
    if ( nAu2c > 0 )
        CovInv_mat( rCnt + 1 : rCnt + nAu2c, rCnt + 1 : rCnt + nAu2c ) = CovInv_obj.Au2c.val;
        rCnt = rCnt + nAu2c;
    end
    
    %% fill in Tu2c
    if ( nTu2c > 0 )
        CovInv_mat( rCnt + 1 : rCnt + nTu2c, rCnt + 1 : rCnt + nTu2c ) = CovInv_obj.Tu2c.val;
        rCnt = rCnt + nTu2c;
    end
    
    %% fill in Bf
    if ( nBf > 0 )
        CovInv_mat( rCnt + 1 : rCnt + nBf, rCnt + 1 : rCnt + nBf ) = CovInv_obj.Bf.val;
        rCnt = rCnt + nBf;
    end
    
    %% fill in Bw
    if ( nBw > 0 )
        CovInv_mat( rCnt + 1 : rCnt + nBw, rCnt + 1 : rCnt + nBw ) = CovInv_obj.Bw.val;
        rCnt = rCnt + nBw;                
    end
    
return

    