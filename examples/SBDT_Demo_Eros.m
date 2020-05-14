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

clear, clc, close all, run ../startup.m  % refresh 

% INITIALIZE DEMO -------------------------------------------------------------

disp( '---------------------------------------------------------------' );
fprintf( 'Starting SBDT_Demo_Eros.\n\n' );

addpath(genpath("../utilities"));
addpath(genpath("../small_body_dynamics/EROS 433"));

% First, make sure the SBDT and the constants structure have been loaded
% using 'addSBDT'.
constants = initialize_SBDT();


% LOAD Eros MODELS ---------------------------------------------------------

% Load pointmass Eros model
% - Orbit model is irrelevant for 2-body dynamics
% - Spherical shape model used
% - Pointmass gravity used
eros_pointmass = loadEros( constants, 1, 1, 1, 1 );

% Load ellipsoid Eros model
% - Orbit model is irrelevant for 2-body dynamics
% - Tri-axial ellipsoid shape model used
% - Constant density gravity used
eros_ellip = loadEros( constants, 1, 1, 2, 2 );

% Load Polyhedral Eros model
% - Orbit model is irrelevant for 2-body dynamics
% - 1708-vertex Polyhedron shape model used
% - Constant density gravity used
eros_poly_1708 = loadEros( constants, 1, 1, 3, 2 );

% Load Polyhedral Eros model
% - Orbit model is irrelevant for 2-body dynamics
% - 10152-vertex Polyhedron shape model used
% - Constant density gravity used
eros_poly_10152 = loadEros( constants, 1, 1, 4, 2 );

% Load SBDT Spherical Harmonic Eros model
% - Orbit model is irrelevant for 2-body dynamics
% - 10152-vertex Polyhedron shape model used
% - 12x12 spherical harmonic gravity field
eros_sphHarm_n15a_sbdt = loadEros( constants, 1, 1, 4, 3 );


eros_sphHarm_n393_sbdt = loadEros( constants, 1, 1, 4, 4 );

% Load Sun model
sun = loadSun(constants, 1, 1, 1, 1);

% COMPUTE GRAVITY ---------------------------------------------------------

% Position to compute gravity at
rp = [ 30, -0.1, -0.3 ]';   % 3x1 (km) in the body fixed frame 

% Compute potential for pointmass model
[ acc_pm, jac_pm, lap_pm, pot_pm ] = potential( rp, constants, ...
                                   eros_pointmass.shape, ...
                                   eros_pointmass.gravity );

% Compute potential for ellipsoid model
[ acc_ell, jac_ell, lap_ell, pot_ell ] = potential( rp, constants, ...
                                   eros_ellip.shape, ...
                                   eros_ellip.gravity );

% Compute potential for constant density polyhedron model
[ acc_poly_1708, jac_poly_1708, lap_poly_1708, pot_poly_1708 ] = ...
                                   potential( rp, constants, ...
                                   eros_poly_1708.shape, ...
                                   eros_poly_1708.gravity );

% Compute potential for constant density polyhedron model
[ acc_poly_10152, jac_poly_10152, lap_poly_10152, pot_poly_10152 ] = ...
                                   potential( rp, constants, ...
                                   eros_poly_10152.shape, ...
                                   eros_poly_10152.gravity );

% Compute potential for spherical harmonic models
[ acc_sh15a_sbdt, jac_sh_sbdt, lap_sh_sbdt, pot_sh_sbdt ] = potential( ...
    rp, constants, eros_sphHarm_n15a_sbdt.shape, ...
    eros_sphHarm_n15a_sbdt.gravity );

[ acc_sh393_sbdt, jac_sh_sbdt, lap_sh_sbdt, pot_sh_sbdt ] = potential( ...
    rp, constants, eros_sphHarm_n393_sbdt.shape, ...
    eros_sphHarm_n15a_sbdt.gravity );


% COMPARE RESULTS -------------------------------------------------------------

% Dump results to the screen
disp( '---------------------------------------------------------------' );
fprintf( 'Accelerations:          X               Y                Z\n' );
fprintf( 'Pointmass:       %8.7d,  %8.7d,  %8.7d\n', acc_pm(1), ...
               acc_pm(2), acc_pm(3) );
fprintf( 'Elipsoid:        %8.7d,  %8.7d,  %8.7d\n', acc_ell(1), ...
               acc_ell(2), acc_ell(3) );
fprintf( 'Poly (1708):     %8.7d,  %8.7d,  %8.7d\n', ...
               acc_poly_1708(1), acc_poly_1708(2), acc_poly_1708(3) );
fprintf( 'Poly (10152):    %8.7d,  %8.7d,  %8.7d\n', ...
               acc_poly_10152(1), acc_poly_10152(2), acc_poly_10152(3) );
fprintf( 'Harmonic (n15a):  %8.7d,  %8.7d,  %8.7d\n', acc_sh15a_sbdt(1), ...
               acc_sh15a_sbdt(2), acc_sh15a_sbdt(3) );
fprintf( 'Harmonic (n393):  %8.7d,  %8.7d,  %8.7d\n', acc_sh393_sbdt(1), ...
               acc_sh393_sbdt(2), acc_sh393_sbdt(3) );

disp( '---------------------------------------------------------------' );


% INTEGRATE IN BODY-FIXED FRAME -----------------------------------------------

% Integration parameters  (IC for a 2:1 periodic orbit in the harmonic model)
tspan = [ 0,  86400 ];  % Integration stop/start in seconds
                                        % past epoch
% pos0 = [ 30, 30, 0 ]';
% vel0 = [ 0, 0, ...
%          sqrt( eros_sphHarm_sbdt.gravity.gm / norm( pos0 ) ) ]' ...
%          - cross( [ 0, 0, eros_sphHarm_sbdt.bodyFrame.pm.w ]', ...
%                   pos0 );
              
sc_location = [25;0;0];
sc_radius = norm(sc_location);
GM = eros_sphHarm_n15a_sbdt.gravity.gm;
sc_orbital_vel = sqrt(GM/sc_radius);
sc_vel = [0; 0; sc_orbital_vel;];

scState0 = [ sc_location; sc_vel ];
simControls = [];

% Integrate
fprintf( 'Integrating with pointmass model...\n\n' );
[ t_pm, scState_pm, ~, JC_pm, ~ ] = ...
    Rot2BP_wPartials( tspan, scState0, [], constants, ...
                           {sun;eros_pointmass}, [], [], simControls );
                       
fprintf( 'Integrating with ellipsoid model...\n\n' );
[ t_ell, scState_ell, ~, JC_ell, ~ ] = ...
    Rot2BP_wPartials( tspan, scState0, [], constants, ...
                           {sun;eros_ellip}, [], [], simControls );
                       
fprintf( 'Integrating with spherical harmonic model n15a (SBDT)...\n\n' );
[ t_sh, scState_sh_n15a, ~, JC_sh, ~ ] = ...
    Rot2BP_wPartials( tspan, scState0, [], constants, ...
                           {sun;eros_sphHarm_n15a_sbdt}, [], [], simControls );
                       
fprintf( 'Integrating with spherical harmonic model n393 (SBDT)...\n\n' );
[ t_sh, scState_sh_n393, ~, JC_sh, ~ ] = ...
    Rot2BP_wPartials( tspan, scState0, [], constants, ...
                           {sun;eros_sphHarm_n393_sbdt}, [], [], simControls );

fprintf( 'Integrating with 1708-vertex polyhedron model...\n\n' );
[ t_poly_1708, scState_poly_1708, ~, JC_poly_1708, ~ ] = ...
    Rot2BP_wPartials( tspan, scState0, [], constants, ...
                           {sun;eros_poly_1708}, [], [], simControls );

fprintf( 'Integrating with 10152-vertex polyhedron model...\n\n' );
[ t_poly_10152, scState_poly_10152, ~, JC_poly_10152, ~ ] = ...
    Rot2BP_wPartials( tspan, scState0, [], constants, ...
                           {sun;eros_poly_10152}, [], [], simControls );
                   
% PLOT IN BODY-FIXED FRAME ----------------------------------------------------

disp( '---------------------------------------------------------------' );
fprintf( 'Plotting...\n\n' );

% Plot polyhedron shape
[ F1, P1 ] = plotPolyhedron( eros_poly_1708.shape, [] );
hold on;

% Plot trajectories
plot3( scState_pm( 1, : ), scState_pm( 2, : ), scState_pm( 3, : ), ...
       'g'  );
plot3( scState_poly_1708( 1, : ), scState_poly_1708( 2, : ), ...
       scState_poly_1708( 3, : ),  'r' );
plot3( scState_poly_10152( 1, : ), scState_poly_10152( 2, : ), ...
       scState_poly_10152( 3, : ),  'c' );
plot3( scState_ell( 1, : ), scState_ell( 2, : ), scState_ell( 3, : ), ...
       'm'  );
plot3( scState_sh_n15a( 1, : ), scState_sh_n15a( 2, : ), scState_sh_n15a( 3, : ), ...
       'b'  );
plot3( scState_sh_n393( 1, : ), scState_sh_n393( 2, : ), scState_sh_n393( 3, : ), ...
       'b--'  );
   
% Finalize plot
legend( 'Eros', 'Pointmass', '1708 Polyhedron', ...
    '10152 Polyhedron', 'Ellipsoid', 'SphHarm (n15a)', 'SphHarm (n393)' );

axis equal;
xlabel( 'X (km)' );
ylabel( 'Y (km)' );
zlabel( 'Z (km)' );
view( 112, 12 );


% Try with SHGI_SBDT


% FINISH DEMO -----------------------------------------------------------------

disp( '---------------------------------------------------------------' );
fprintf( 'End of demo_eros.\n\n' );
disp( '---------------------------------------------------------------' );


% -----------------------------------------------------------------------------

cspice_kclear % This cleares the SPICE files from Matlab's memory