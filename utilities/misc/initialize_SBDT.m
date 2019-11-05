%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2019 by California Institute of Technology.  ALL RIGHTS RESERVED. %
% United  States  Government  sponsorship  acknowledged.   Any commercial use %
% must   be  negotiated  with  the  Office  of  Technology  Transfer  at  the %
% California Institute of Technology.                                         %
%                                                                             %
% This software may be subject to  U.S. export control laws  and regulations. %
% By accepting this document,  the user agrees to comply  with all applicable %
% U.S. export laws and regulations.  User  has the responsibility  to  obtain %
% export  licenses,  or  other  export  authority  as may be required  before %
% exporting  such  information  to  foreign  countries or providing access to %
% foreign persons.                                                            %
%                                                                             %
% This  software  is a copy  and  may not be current.  The latest  version is %
% maintained by and may be obtained from the Mobility  and  Robotics  Sytstem %
% Section (347) at the Jet  Propulsion  Laboratory.   Suggestions and patches %
% are welcome and should be sent to the software's maintainer.                %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [constants] = initialize_SBDT()
%INITIALIZE_SBDT 
%   Assumes that SBDT_PATH is specified as a variable somewhere.
%   If it isn't, defaults to a directory on Federico's computer.
global SBDT_PATH
% Attempt to grab the SBDT path from an environment variable
if isempty(SBDT_PATH)
    SBDT_PATH=getenv("SBDT_PATH");
end

if isempty(SBDT_PATH)
    warning("SBDT_PATH environment variable not found. Assuming that SBDT is installed in the same folder as icc-dev")
    SBDT_PATH =  strcat(pwd, '/../SBDT') ; 
end

addpath(fullfile(SBDT_PATH,'Startup'));
userModelsPath = fullfile(SBDT_PATH,'ExampleUserModels');
constantsModel = 1;
global constants;
constants = addSBDT(SBDT_PATH, userModelsPath, constantsModel);

end