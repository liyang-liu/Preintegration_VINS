
if(InertialDelta_options.bMalaga == 1)
    Data_config.imufulldata = IMUparking6L;
elseif(InertialDelta_options.bDinuka == 1)
    display('bDinuka');
    Data_config.DATA_DIR = ['..' filesep 'Data' filesep 'Dinuka' filesep 'dataset_19_10_15' filesep];%dataset_19_10_15
    Data_config.imgdir = Data_config.DATA_DIR;
    Data_config.imufulldir = [Data_config.DATA_DIR 'imudata_nonoise.mat'];% small imudata_nonoise['.' filesep 'Malaga' filesep 'IMUrawData.mat'];
    Data_config.gtVelfulldir = [Data_config.DATA_DIR 'velocity_ground_truth.mat'];
    Data_config.gtFile = [Data_config.DATA_DIR 'gtIMUposes.mat'];    
end
