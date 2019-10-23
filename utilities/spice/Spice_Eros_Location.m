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

% A simple demo script that showcases NAIF's capabilities

clear all; close all; clc;

%Where the NAIF folder is.
naif_path = getenv("NAIF_PATH");
if isempty(naif_path)
    warning("Environment variable NAIF_PATH not set. Assuming NAIF files were downloaded in the current folder")
    naif_path=pwd;
end
% Load the appropriate SPICE kernels
load_spice_eros(naif_path);

% Compute the position of Eros for a bunch of time steps
STEP = 1000;
et = cspice_str2et( {'Jun 20, 2004', 'Dec 1, 2005'} );
times      = (0:STEP-1) * ( et(2) - et(1) )/STEP + et(1);

% Position of Eros wrt Sun in J2000 frame
state_Eros = cspice_spkpos('2000433',times, 'J2000', 'NONE', 'SUN' );
state_Earth = cspice_spkpos('Earth',times, 'J2000', 'NONE', 'SUN' );
figure()
%subplot(1,3,1)
plot3(state_Eros(1,:),state_Eros(2,:),state_Eros(3,:))
hold all
plot3(state_Earth(1,:),state_Earth(2,:),state_Earth(3,:))
title('Eros and Earth wrt Sun in J2K')
legend('433 Eros', 'Earth')
axis equal

% State (position+velocity) of Sun wrt Eros in J2000 frame
[state, lt] = cspice_spkezr('SUN', times, 'J2000', 'NONE', '2000433');
%subplot(1,3,2)
figure()
plot3(state(1,:),state(2,:),state(3,:))
title('Sun wrt Eros in J2K')
axis equal

% State (position+velocity) of Sun wrt Eros in Eros frame
[state, lt] = cspice_spkezr('SUN', times, 'IAU_EROS', 'NONE', '2000433');
%subplot(1,3,3)
figure()
plot3(state(1,:),state(2,:),state(3,:))
title('Sun wrt Eros in IAU\_Eros')
axis equal