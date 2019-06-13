% Load SBDT
% Assumes that SBDT_PATH is specified as a variable somewhere.
% If it isn't, defaults to a directory on Federico's computer.

function [] = initialize_SBDT()
global SBDT_PATH
if isempty(SBDT_PATH)
    SBDT_PATH = '/Users/frossi/Documents/JPL/ICC/SBDT';
end

addpath(strcat(SBDT_PATH,'/Startup'));
userModelsPath = strcat(SBDT_PATH,'/ExampleUserModels');
constantsModel = 1;
constants = addSBDT(SBDT_PATH, userModelsPath, constantsModel); %#ok<NASGU>

end