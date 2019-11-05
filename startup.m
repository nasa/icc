%% Configure properties for this machine 

% Store location of rootpath as global 
global ROOT_PATH
ROOT_PATH = pwd; 

% Store location of SBDT as global
global SBDT_PATH
SBDT_PATH = getenv("SBDT_PATH");
if isempty(SBDT_PATH)
    warning("SBDT_PATH environment variable not found. Assuming that SBDT is installed in the same folder as icc-dev")
    SBDT_PATH =  strcat(ROOT_PATH, '/../SBDT') ; 
end

% Store location of NAIF as global
global NAIF_PATH
NAIF_PATH =  strcat(ROOT_PATH, '/utilities/spice') ; 

% Add MICE to path 
addpath(genpath(strcat(ROOT_PATH, '/../mice/'))); 

%% Complete Setup 

run utilities/misc/initialize_SBDT.m
addpath(ROOT_PATH); 
clc