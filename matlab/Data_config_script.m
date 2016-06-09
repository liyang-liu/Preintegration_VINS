global Data_config

Data_config.BASE_DIR = pwd;

if( PreIntegration_options.bSimData )
    
    Data_config.imgdir          = [];
    Data_config.DATA_DIR        = [ '..' filesep 'Data' filesep 'Simulation' filesep ];
    
elseif(PreIntegration_options.bMalaga == 1)
    
    Data_config.DATA_ROOT       =   [ '..' filesep 'Data' filesep 'Malaga' filesep ];
    Data_config.DATA_DIR        =   [ Data_config.DATA_ROOT 'Whole170R' filesep 'Result' filesep ];
    Data_config.imgdir          =   [ Data_config.DATA_ROOT 'Whole170R' filesep ];
    Data_config.imufulldir      =	[ Data_config.DATA_ROOT 'IMUrawData.mat' ];
    Data_config.gtFile          =   [ Data_config.DATA_ROOT 'Whole170R' filesep 'GT_P0_PA.mat' ];    
    load( Data_config.imufulldir );
    Data_config.imufulldata     =   IMUparking6L;
    
elseif(PreIntegration_options.bDinuka == 1)
    
    display('bDinuka');
    Data_config.DATA_DIR        = [ '..' filesep 'Data' filesep 'Dinuka' filesep 'dataset_19_10_15' filesep ]; %dataset_19_10_15
    Data_config.imgdir          = Data_config.DATA_DIR;
    Data_config.imufulldir      = [ Data_config.DATA_DIR 'imudata_nonoise.mat' ]; % small imudata_nonoise['.' filesep 'Malaga' filesep 'IMUrawData.mat'];
    Data_config.gtVelfulldir    = [ Data_config.DATA_DIR 'velocity_ground_truth.mat' ];
    Data_config.gtFile          = [ Data_config.DATA_DIR 'gtIMUposes.mat' ];    
end


% create temporary working directory
Data_config.TEMP_DIR = [ Data_config.BASE_DIR filesep '..' filesep 'temp' filesep ];
if ( ~exist( Data_config.TEMP_DIR, 'dir') )
    mkdir ( Data_config.TEMP_DIR )
end
display( [' !!! Temporary working directory: \"' Data_config.TEMP_DIR '\" !!!' ] );

% create result dir
Data_config.RESULT_DIR = [ Data_config.BASE_DIR filesep '..' filesep 'results' filesep ];
if ( ~exist( Data_config.RESULT_DIR, 'dir') )
    mkdir ( Data_config.RESULT_DIR )
end
display( [' !!! Result directory: \"' Data_config.RESULT_DIR '\" !!!' ] );
