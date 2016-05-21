function CovMatrixInv = SLAM_CalcCovMatrixInv( SLAM_Params, Zobs, Rd )

    CovInv_obj = SLAM_CovInv_Init( SLAM_Params, Zobs, Rd );
    
    nRow = Zobs.Bw.row(end); %matrix dim    
    CovMatrixInv = SLAM_CovInv_Object2Matrix( CovInv_obj, nRow );

    return;
        

    
