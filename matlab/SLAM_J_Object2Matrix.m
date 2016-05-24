function [J_mat] = JObject2Matrix( J_obj, Zobs, X_obj )

    global PreIntegration_options

    %display('JObject2Matrix');
    
    J_vec = [];
    J_row = [];
    J_col = [];
    
    %% dUv_dX
    numUV = length( J_obj.dUv_dX );
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
        
        %dATu2c
        J_row = [ J_row; J_obj.dUv_dX(i).dATu2c.row(:) ];
        J_col = [ J_col; J_obj.dUv_dX(i).dATu2c.col(:) ];
        J_vec = [ J_vec; J_obj.dUv_dX(i).dATu2c.val(:) ];        
    end
    
    if ( PreIntegration_options.bPreInt == 1 )
        %% dIntlDelta_dX
        numIntlDelta = length( J_obj.dIntlDelta_dX );
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

            % dDp_dV_1
            J_row = [ J_row; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dV_1.row(:) ];
            J_col = [ J_col; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dV_1.col(:) ];
            J_vec = [ J_vec; J_obj.dIntlDelta_dX(i).dDp_dX.dDp_dV_1.val(:) ];            

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

            %dDphi_dBw
            J_row = [ J_row; J_obj.dIntlDelta_dX(i).dDphi_dX.dDphi_dBw.row(:) ];
            J_col = [ J_col; J_obj.dIntlDelta_dX(i).dDphi_dX.dDphi_dBw.col(:) ];
            J_vec = [ J_vec; J_obj.dIntlDelta_dX(i).dDphi_dX.dDphi_dBw.val(:) ];
        end % for
        
    else % bPreInt == 0
        numImu = length( J_obj.dImu_dX );
        
        for i = 1 : numImu
            %% dW_dX
            if ( ~isempty( J_obj.dImu_dX(i).dW_dX.dW_dPhi ) )
                % d_w[ij] / d_A[ij]
                J_row = [ J_row; J_obj.dImu_dX(i).dW_dX.dW_dPhi.row(:) ];
                J_col = [ J_col; J_obj.dImu_dX(i).dW_dX.dW_dPhi.col(:) ];
                J_vec = [ J_vec; J_obj.dImu_dX(i).dW_dX.dW_dPhi.val(:) ];            
            end
            % d_w[ij] / d_A[i+1,j]
            J_row = [ J_row; J_obj.dImu_dX(i).dW_dX.dW_dPhi_1.row(:) ];
            J_col = [ J_col; J_obj.dImu_dX(i).dW_dX.dW_dPhi_1.col(:) ];
            J_vec = [ J_vec; J_obj.dImu_dX(i).dW_dX.dW_dPhi_1.val(:) ];            
            % d_w[ij] / d_bw
            J_row = [ J_row; J_obj.dImu_dX(i).dW_dX.dW_dBw.row(:) ];
            J_col = [ J_col; J_obj.dImu_dX(i).dW_dX.dW_dBw.col(:) ];
            J_vec = [ J_vec; J_obj.dImu_dX(i).dW_dX.dW_dBw.val(:) ];            

            %% dAcc_dX            
            if ( ~isempty( J_obj.dImu_dX(i).dAcc_dX.dAcc_dPhi ) )
                % dAcc_dPhi
                J_row = [ J_row; J_obj.dImu_dX(i).dAcc_dX.dAcc_dPhi.row(:) ];
                J_col = [ J_col; J_obj.dImu_dX(i).dAcc_dX.dAcc_dPhi.col(:) ];
                J_vec = [ J_vec; J_obj.dImu_dX(i).dAcc_dX.dAcc_dPhi.val(:) ];                
            end
            % d_a[ij] / d_v[ij]
            J_row = [ J_row; J_obj.dImu_dX(i).dAcc_dX.dAcc_dV.row(:) ];
            J_col = [ J_col; J_obj.dImu_dX(i).dAcc_dX.dAcc_dV.col(:) ];
            J_vec = [ J_vec; J_obj.dImu_dX(i).dAcc_dX.dAcc_dV.val(:) ];                
            %  d_a[ij] / d_v[i+1,j]
            J_row = [ J_row; J_obj.dImu_dX(i).dAcc_dX.dAcc_dV_1.row(:) ];
            J_col = [ J_col; J_obj.dImu_dX(i).dAcc_dX.dAcc_dV_1.col(:) ];
            J_vec = [ J_vec; J_obj.dImu_dX(i).dAcc_dX.dAcc_dV_1.val(:) ];
            % d_a[ij] / d_g
            J_row = [ J_row; J_obj.dImu_dX(i).dAcc_dX.dAcc_dG.row(:) ];
            J_col = [ J_col; J_obj.dImu_dX(i).dAcc_dX.dAcc_dG.col(:) ];
            J_vec = [ J_vec; J_obj.dImu_dX(i).dAcc_dX.dAcc_dG.val(:) ];            
            % d_a[ij] / d_bf
            J_row = [ J_row; J_obj.dImu_dX(i).dAcc_dX.dAcc_dBf.row(:) ];
            J_col = [ J_col; J_obj.dImu_dX(i).dAcc_dX.dAcc_dBf.col(:) ];
            J_vec = [ J_vec; J_obj.dImu_dX(i).dAcc_dX.dAcc_dBf.val(:) ];
        
            %% dDeltaT_dX
            % d_bZ[ij] / d_T[i,j]
            if ( ~isempty( J_obj.dImu_dX(i).dDeltaT_dX.dDeltaT_dT ) )
                J_row = [ J_row; J_obj.dImu_dX(i).dDeltaT_dX.dDeltaT_dT.row(:) ];
                J_col = [ J_col; J_obj.dImu_dX(i).dDeltaT_dX.dDeltaT_dT.col(:) ];
                J_vec = [ J_vec; J_obj.dImu_dX(i).dDeltaT_dX.dDeltaT_dT.val(:) ];
            end
            % d_bZ[ij] / d_T[i+1,j]
            J_row = [ J_row; J_obj.dImu_dX(i).dDeltaT_dX.dDeltaT_dT_1.row(:) ];
            J_col = [ J_col; J_obj.dImu_dX(i).dDeltaT_dX.dDeltaT_dT_1.col(:) ];
            J_vec = [ J_vec; J_obj.dImu_dX(i).dDeltaT_dX.dDeltaT_dT_1.val(:) ];
            
            % d_bZ[ij] / d_v[i,j]
            J_row = [ J_row; J_obj.dImu_dX(i).dDeltaT_dX.dDeltaT_dV.row(:) ];
            J_col = [ J_col; J_obj.dImu_dX(i).dDeltaT_dX.dDeltaT_dV.col(:) ];
            J_vec = [ J_vec; J_obj.dImu_dX(i).dDeltaT_dX.dDeltaT_dV.val(:) ];
        end %for
    end % if bPreInt
    
    if ( PreIntegration_options.bAddZg == 1 )
        J_row = [ J_row; J_obj.dG_dG.row(:) ];
        J_col = [ J_col; J_obj.dG_dG.col(:) ];
        J_vec = [ J_vec; J_obj.dG_dG.val(:) ];            
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
    
    %fprintf('J_row.len = %d, J_col.len = %d, J_vec.len=%d\n', ...
    %               length(J_row), length(J_col), length(J_vec));
    
    M = Zobs.Bw.row(end);
    N = X_obj.Bw.col(end);
    
    J_mat = sparse( J_row, J_col, J_vec, M, N );
    