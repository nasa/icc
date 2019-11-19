%% Configure properties for this machine 

% Store location of rootpath
ROOT_PATH = pwd; 

% Load location of SBDT from environment variable
SBDT_PATH = getenv("SBDT_PATH");
if isempty(SBDT_PATH)
    warning("SBDT_PATH environment variable not found. Assuming that SBDT is installed in the same folder as icc-dev")
    SBDT_PATH =  strcat(ROOT_PATH, '/../SBDT') ; 
end
addpath(genpath(SBDT_PATH));

% Load location of NAIF from environment variable
NAIF_PATH = getenv("NAIF_PATH");
if isempty(NAIF_PATH)
    warning("NAIF_PATH environment variable not found. "+...
    "Assuming that NAIF data is stored in $ROOT_PATH/utilities/spice."+ ...
    "This is likely to cause an error when running parfor "+ ...
    "- you should really specify the NAIF path as an environment variable.")
    NAIF_PATH = strcat(ROOT_PATH, '/utilities/spice') ; 
end
addpath(genpath(NAIF_PATH));

% Add MICE to path 
addpath(genpath(strcat(ROOT_PATH, '/../mice/'))); 

%% Check whether SBDT exists

if ~exist('addSBDT','file')
    error("JPL's Small Body Dynamics Toolkit (SBDT) is required to run this example. See the README for how to get access to SBDT.")
end

%% Set up CVX to avoid using SDPT3
if ~exist('cvx_solver', 'file')
    error("CVX is a required dependency. Please download CVX from cvxr.com")
end

if strcmp(cvx_solver,'SDPT3')
    warning("SDPT3 is known to falsely return infeasible for our problem. Switching CVX solver to SeDuMi")
    cvx_solver SeDuMi
end

%% Complete Setup 

evalc("run utilities/misc/initialize_SBDT.m");
addpath(ROOT_PATH); 