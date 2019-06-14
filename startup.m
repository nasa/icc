%% Configure properties for this machine 

% Store location of rootpath as global 
global ROOT_PATH
ROOT_PATH = pwd; 

% Store location of SBDT as global
global SBDT_PATH
SBDT_PATH =  strcat(ROOT_PATH, '/../SBDT') ; 

%% Complete Setup 

run utilities/misc/initialize_SBDT.m
addpath(ROOT_PATH); 
clc