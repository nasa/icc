%% Configure properties for this machine 

% Store location of rootpath as global 
global ROOT_PATH
ROOT_PATH = pwd; 

% Store location of SBDT as global
global SBDT_PATH
SBDT_PATH =  strcat(ROOT_PATH, '/../SBDT') ; 

% Store location of NAIF as global
global NAIF_PATH
% NAIF_PATH =  strcat(ROOT_PATH, '/utilities/spice') ; 
NAIF_PATH = strcat('/Users/bandyopa/JPL Work/Amir JP ICC 2018/NAIF_PATH');

% Add MICE to path 
addpath(genpath(strcat(ROOT_PATH, '/../mice/'))); 

%% Complete Setup 

run utilities/misc/initialize_SBDT.m
addpath(ROOT_PATH); 
clc