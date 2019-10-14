function state = get_sun_state(times)
%GET_SUN_STATE returns sun state 

%Where the NAIF folder is.
global NAIF_PATH
naif_path = NAIF_PATH;

% Load the appropriate SPICE kernels
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

