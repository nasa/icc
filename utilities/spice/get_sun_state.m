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

function state = get_sun_state(times)
%GET_SUN_STATE returns sun state 

%Where the NAIF folder is.
global NAIF_PATH
naif_path = NAIF_PATH;

% Load the appropriate SPICE kernels
disp("Attempting to load SPICE for Eros from "+naif_path);
if isempty(naif_path)
    warning("Cannot find global NAIF_PATH - are you running a parfor loop?")
    warning("I will try to determine NAIF_PATH from your environment variables")
    naif_path = getenv("NAIF_PATH");
    if isempty(naif_path)
        warning("Cannot find environment variable NAIF_PATH. I will be making a guess as to where NAIF may be.")
        naif_path = fullfile(pwd, '..','utilities','spice') ;
    end
end
load_spice_eros(naif_path);

% Compute the position of Eros for a bunch of time steps
% STEP = 1000;
% et = cspice_str2et( {'Jun 20, 2004', 'Dec 1, 2005'} );
% times      = 1% (0:STEP-1) * ( et(2) - et(1) )/STEP + et(1);

% State (position+velocity) of Sun wrt Eros in Eros frame
[state, ~] = cspice_spkezr('SUN', times, 'IAU_EROS', 'NONE', '2000433');
% subplot(1,3,3)
% plot3(state(1,:),state(2,:),state(3,:))
% title('Sun wrt Eros in IAU\_Eros')
% axis equal


end

