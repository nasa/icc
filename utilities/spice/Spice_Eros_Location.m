clear all; close all; clc;

%Where the NAIF folder is.
naif_path = '/Users/frossi/Documents/MATLAB';
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