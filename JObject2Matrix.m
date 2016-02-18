function [J_mat] = JObject2Matrix( J_obj, Zobs, X_obj )

    global InertialDelta_options

    display('JObject2Matrix');
    
    J_vec = [];
    J_row = [];
    J_col = [];
    
    %% dUv_dX
    numUV = length( J_obj.dUv_dX )
    for i=1:numUV
        if ( ~isempty( J_obj.dUv_dX(i).dAbgxyz_1 ) )
            %dAbgxyz
            J_row = [ J_row; J_obj.dUv_dX(i).dAbgxyz_1.row(:) ];
            J_col = [ J_col; J_obj.dUv_dX(i).dAbgxyz_1.col(:) ];
            J_vec = [ J_vec; J_obj.dUv_dX(i).dAbgxyz_1.val(:) ];        
        end
        
        %dFxyz
        J_row = [ J_row; J_obj.dUv_dX(i).dFxyz.row(:) ];
        J_col = [ J_col; J_obj.dUv_dX(i).dFxyz.col(:) ];
        J_vec = [ J_vec; J_obj.dUv_dX(i).dFxyz.val(:) ];
        
        %dTu2c
        J_row = [ J_row; J_obj.dUv_dX(i).dTu2c.row(:) ];
        J_col = [ J_col; J_obj.dUv_dX(i).dTu2c.col(:) ];
        J_vec = [ J_vec; J_obj.dUv_dX(i).dTu2c.val(:) ];        
    end
    
    %% dIntlDelta_dX
    numIntlDelta = length( J_obj.dIntlDelta_dX )
    for i=1:numIntlDelta
        
        %% dDp_dX
        
        if ( ~isempty( J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dAbg_1 ) )
            %dDp_dAbg_1
            J_row = [ J_row; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dAbg_1.row(:) ];
            J_col = [ J_col; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dAbg_1.col(:) ];
            J_vec = [ J_vec; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dAbg_1.val(:) ];            
        end
    
        if ( ~isempty( J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dTrans_1 ) )
            %dDp_dAbg_1
            J_row = [ J_row; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dTrans_1.row(:) ];
            J_col = [ J_col; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dTrans_1.col(:) ];
            J_vec = [ J_vec; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dTrans_1.val(:) ];            
        end
    
        %dDp_dTrans_2
        J_row = [ J_row; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dTrans_2.row(:) ];
        J_col = [ J_col; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dTrans_2.col(:) ];
        J_vec = [ J_vec; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dTrans_2.val(:) ];            

        %dDp_dG
        J_row = [ J_row; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dG.row(:) ];
        J_col = [ J_col; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dG.col(:) ];
        J_vec = [ J_vec; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dG.val(:) ];            

        %dDp_dBf
        J_row = [ J_row; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dBf.row(:) ];
        J_col = [ J_col; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dBf.col(:) ];
        J_vec = [ J_vec; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dBf.val(:) ];            

        %dDp_dBw
        J_row = [ J_row; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dBw.row(:) ];
        J_col = [ J_col; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dBw.col(:) ];
        J_vec = [ J_vec; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dBw.val(:) ];            
        
        %% dDv_dX
        
        if ( ~isempty( J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dAbg_1 ) )
            %dDv_dAbg_1
            J_row = [ J_row; J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dAbg_1.row(:) ];
            J_col = [ J_col; J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dAbg_1.col(:) ];
            J_vec = [ J_vec; J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dAbg_1.val(:) ];            
        end

        % dDv_dV_1
        J_row = [ J_row; J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dV_1.row(:) ];
        J_col = [ J_col; J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dV_1.col(:) ];
        J_vec = [ J_vec; J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dV_1.val(:) ];
        
        % dDv_dV_2
        J_row = [ J_row; J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dV_2.row(:) ];
        J_col = [ J_col; J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dV_2.col(:) ];
        J_vec = [ J_vec; J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dV_2.val(:) ];
        
        % dDv_dG
        J_row = [ J_row; J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dG.row(:) ];
        J_col = [ J_col; J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dG.col(:) ];
        J_vec = [ J_vec; J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dG.val(:) ];
        
        % dDv_Bf
        J_row = [ J_row; J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dBf.row(:) ];
        J_col = [ J_col; J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dBf.col(:) ];
        J_vec = [ J_vec; J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dBf.val(:) ];

        % dDv_Bw
        J_row = [ J_row; J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dBw.row(:) ];
        J_col = [ J_col; J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dBw.col(:) ];
        J_vec = [ J_vec; J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dBw.val(:) ];
        
        %% dDphi_dX        
        
        if ( ~isempty( J_obj.dIntlDelta_dX(i).dDv_dX.dDv_dAbg_1 ) )
            %dDphi_dAbg_1
            J_row = [ J_row; J_obj.dIntlDelta_dX(i).dDphi_dX.dDphi_dAbg_1.row(:) ];
            J_col = [ J_col; J_obj.dIntlDelta_dX(i).dDphi_dX.dDphi_dAbg_1.col(:) ];
            J_vec = [ J_vec; J_obj.dIntlDelta_dX(i).dDphi_dX.dDphi_dAbg_1.val(:) ];            
        end

        %dDphi_dAbg_2
        J_row = [ J_row; J_obj.dIntlDelta_dX(i).dDphi_dX.dDphi_dAbg_2.row(:) ];
        J_col = [ J_col; J_obj.dIntlDelta_dX(i).dDphi_dX.dDphi_dAbg_2.col(:) ];
        J_vec = [ J_vec; J_obj.dIntlDelta_dX(i).dDphi_dX.dDphi_dAbg_2.val(:) ];            
        
    end
    
    %% dAu2c_dX
    J_row = [ J_row; J_obj.dAu2c_dX.row(:) ];
    J_col = [ J_col; J_obj.dAu2c_dX.col(:) ];
    J_vec = [ J_vec; J_obj.dAu2c_dX.val(:) ];            
        
    %% dTu2c_dX
    J_row = [ J_row; J_obj.dTu2c_dX.row(:) ];
    J_col = [ J_col; J_obj.dTu2c_dX.col(:) ];
    J_vec = [ J_vec; J_obj.dTu2c_dX.val(:) ];            
        
    %% dBf_dX
    J_row = [ J_row; J_obj.dBf_dX.row(:) ];
    J_col = [ J_col; J_obj.dBf_dX.col(:) ];
    J_vec = [ J_vec; J_obj.dBf_dX.val(:) ];            
        
    %% dBf_dw
    J_row = [ J_row; J_obj.dBw_dX.row(:) ];
    J_col = [ J_col; J_obj.dBw_dX.col(:) ];
    J_vec = [ J_vec; J_obj.dBw_dX.val(:) ];            
    
    M = Zobs.Bw.row(end)
    N = X_obj.Bw.col(end)
    
    J_mat = sparse( J_row, J_col, J_vec, M, N );
    