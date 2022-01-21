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

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%       Usage example of optimal Keplerian transfer from Tycho            %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clear, clc, close all, run ../startup.m  % refresh

rng default % Pseudo-random but repeatable scenario

% Add Required Packages to PATH
addpath(genpath(strcat(ROOT_PATH,'/small_body_dynamics/EROS 433')))
addpath(strcat(ROOT_PATH,'/small_body_dynamics'))
addpath(strcat(ROOT_PATH,'/visualization'))
addpath(genpath(strcat(ROOT_PATH,'/utilities'))) % Add all utilities
userModelsPath = strcat(SBDT_PATH,'/ExampleUserModels');
constantsModel = 1;
addpath(strcat(SBDT_PATH,'/Startup'));
constants = addSBDT(SBDT_PATH, userModelsPath, constantsModel);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                       Initialize Eros Model                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

eros_sbdt = loadEros( constants, 1, 1, 4, 3 );
ErosModel = SphericalHarmonicsGravityIntegrator_SBDT(eros_sbdt, constants);

GM = ErosModel.BodyModel.gravity.gm*1e9; % Convert back to m from km.

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Define initial and final orbit around Eros                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

carrier_orbit = initialize_carrier_orbit(ErosModel);
final_orbit = initialize_random_orbits(1, ErosModel); % [m, m/s]

[initial_orbital_params.a, initial_orbital_params.e, initial_orbital_params.i, initial_orbital_params.Omega, initial_orbital_params.omega, initial_orbital_params.theta] = rv2op(carrier_orbit(1:3),carrier_orbit(4:6),GM);

if initial_orbital_params.e <eps
    initial_orbital_params.e = 0.0001;
end
if abs(initial_orbital_params.i) <eps
    initial_orbital_params.i = 0.0001;
end

[final_orbital_params.a, final_orbital_params.e, final_orbital_params.i, final_orbital_params.Omega, final_orbital_params.omega, final_orbital_params.theta] = rv2op(final_orbit(1:3),final_orbit(4:6),GM);

if final_orbital_params.e <eps
    final_orbital_params.e = 0.0001;
end
if abs(final_orbital_params.i) <eps
    final_orbital_params.i = 0.0001;
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Assess possible transfer strategies                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[ordered_delta_vs, ordered_transfer_times, ordered_intermediate_orbits, ordered_manoeuvers] = optimal_keplerian_orbit_transfer(initial_orbital_params, final_orbital_params, GM);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                             Plot the result                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

plot_asteroid = @() render_asteroid_3d(ErosModel, true, 0,1e3);

plt_rows = floor(sqrt(length(ordered_intermediate_orbits)));
plt_columns = ceil(length(ordered_intermediate_orbits)/plt_rows);

figure()
for option_ix = 1:length(ordered_intermediate_orbits)
   axes = subplot(plt_rows, plt_columns, option_ix);
   intermediate_orbits = ordered_intermediate_orbits{option_ix};
   intermediate_orbits = {intermediate_orbits{2:end}}; % The first orbit is just the initial orbit
   manoeuvers = ordered_manoeuvers{option_ix};
   plot_transfer(initial_orbital_params, final_orbital_params, intermediate_orbits, manoeuvers, GM, axes, plot_asteroid);
   title({sprintf("\\fontsize{12}Strategy %d", option_ix), sprintf("\\fontsize{9}dV=%.2f", ordered_delta_vs(option_ix)), sprintf("\\fontsize{9}dT=%.2f",ordered_transfer_times(option_ix))});
    view(3)
    axis equal
end

plot_pareto_curve(initial_orbital_params, final_orbital_params, ordered_intermediate_orbits, ordered_transfer_times, ordered_delta_vs, ordered_manoeuvers, GM, "Eros", plot_asteroid);
% 
% figure()
% plot(ordered_transfer_times, ordered_delta_vs, '.');
% hold all
% for option_ix = 1:length(ordered_intermediate_orbits)
%     text(ordered_transfer_times(option_ix), ordered_delta_vs(option_ix), sprintf("Strategy %d", option_ix));
% end
% xlabel('Transfer time (s)')
% ylabel('deltaV (m/s)')