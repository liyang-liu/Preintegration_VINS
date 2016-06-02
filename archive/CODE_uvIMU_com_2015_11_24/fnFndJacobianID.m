function [idRow, idCol, nJacs] = fnFndJacobianID(nIMUdata, bUVonly, ...
    bPreInt, nPoses, RptFeatureObs, ImuTimestamps)

nObsId_FeatureObs = 2;
nFeatures = size(RptFeatureObs, 1);
idRow = [];
idCol = [];
nJacs = 0;
nUV = 0;

for fid=1:nFeatures
    nObs = RptFeatureObs(fid, nObsId_FeatureObs);
    for(oid=1:nObs)
        pid = RptFeatureObs(fid, oid*3); 
        if(pid > nPoses)
            break;
        end        
        if(bUVonly == 1)
            idPbase = 6*(pid-2);
            idFbase = 6*(nPoses-1);
            idATu2cbase = 6*(nPoses-1)+3*nFeatures;
        elseif(bPreInt == 1)
            idPbase = 6*(pid-2);
            idFbase = 6*(nPoses-1);
            idATu2cbase = 6*(nPoses-1)+3*nFeatures+3*nPoses+3;
        else
            idPbase = (ImuTimestamps(pid)-ImuTimestamps(1)-1)*6;
            idFbase = 6*nIMUdata;%nIMUrate*(nPoses-1) 
            idATu2cbase = 6*nIMUdata+3*nFeatures+3*(nIMUdata+1)+3;
        end        
        if(pid > 1)%       da,db,dg,dxc,dyc,dzc, dxf,dyf,dzf, (da,db,dg,dx,dy,dz)u2c
            idRow = [idRow, nUV+1,nUV+2,nUV+1,nUV+2,nUV+1,nUV+2, ...
                        nUV+1,nUV+2,nUV+1,nUV+2,nUV+1,nUV+2, ...
                    nUV+1,nUV+2,nUV+1,nUV+2,nUV+1,nUV+2, ...
                    nUV+1,nUV+2,nUV+1,nUV+2,nUV+1,nUV+2, ...
                        nUV+1,nUV+2,nUV+1,nUV+2,nUV+1,nUV+2
                    ];
            idCol = [idCol, (idPbase+1),(idPbase+1),(idPbase+2),(idPbase+2),...
                        (idPbase+3),(idPbase+3),(idPbase+4),(idPbase+4),...
                        (idPbase+5),(idPbase+5),(idPbase+6),(idPbase+6),...
                     idFbase+3*(fid-1)+1,idFbase+3*(fid-1)+1,...
                        idFbase+3*(fid-1)+2,idFbase+3*(fid-1)+2,...
                        idFbase+3*(fid-1)+3,idFbase+3*(fid-1)+3,...
                     idATu2cbase+1,idATu2cbase+1,idATu2cbase+2,idATu2cbase+2,...
                        idATu2cbase+3,idATu2cbase+3,idATu2cbase+4,idATu2cbase+4,...
                        idATu2cbase+5,idATu2cbase+5,idATu2cbase+6,idATu2cbase+6                        
                        ];
             nJacs = nJacs + 30;
        else% pid == 1,     dxf,dyf,dzf, (da,db,dg,dx,dy,dz)u2c
            idRow = [idRow, nUV+1,nUV+2,nUV+1,nUV+2,nUV+1,nUV+2, ...
                    nUV+1,nUV+2,nUV+1,nUV+2,nUV+1,nUV+2, ...
                        nUV+1,nUV+2,nUV+1,nUV+2,nUV+1,nUV+2
                    ];
            idCol = [idCol, idFbase+3*(fid-1)+1,idFbase+3*(fid-1)+1,...
                        idFbase+3*(fid-1)+2,idFbase+3*(fid-1)+2,...
                        idFbase+3*(fid-1)+3,idFbase+3*(fid-1)+3,...
                     idATu2cbase+1,idATu2cbase+1,idATu2cbase+2,idATu2cbase+2,...
                        idATu2cbase+3,idATu2cbase+3,idATu2cbase+4,idATu2cbase+4,...
                        idATu2cbase+5,idATu2cbase+5,idATu2cbase+6,idATu2cbase+6
                        ]; 
            nJacs = nJacs + 18;           
        end
        nUV = nUV + 2;    
    end
end


