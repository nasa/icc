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

% Load EROS path
function [] = load_spice_eros(NAIF_PATH)
if nargin<1
    NAIF_PATH = pwd;
end
cspice_furnsh(fullfile(NAIF_PATH,'naif','generic_kernels','lsk','naif0012.tls'));
cspice_furnsh(fullfile(NAIF_PATH,'/naif','generic_kernels','pck','pck00010.tpc'));
cspice_furnsh(fullfile(NAIF_PATH,'/naif','generic_kernels','spk','planets','de435.bsp'));

cspice_furnsh(fullfile(NAIF_PATH,'/naif','generic_kernels','pck','gm_de431.tpc'));

%cspice_furnsh(strcat(NAIF_PATH,'/naif/generic_kernels/spk/asteroids/codes_300ast_20100725.bsp'));
cspice_furnsh(fullfile(NAIF_PATH,'/naif','generic_kernels','spk','asteroids','a433.bsp'));