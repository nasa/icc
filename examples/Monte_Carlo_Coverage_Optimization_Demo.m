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
%           Usage example of Monte Carlo Coverage Optimizer               %
%                                                                         %
% Demonstrates the usage of the monte_carlo_coverage_optimizer module by  %
% finding the best orbits from a sequential Monte Carlo. Results are      %
% simulated using the visualization utilities.                            %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear, clc, close all, run ../startup.m  % refresh

% Add Required Packages to PATH
addpath(genpath(strcat(ROOT_PATH,'/small_body_dynamics/EROS 433')))
addpath(strcat(ROOT_PATH,'/small_body_dynamics'))
addpath(genpath(strcat(ROOT_PATH,'/utilities'))) % Add all utilities
addpath(strcat(ROOT_PATH,'/visualization'))
addpath(strcat(ROOT_PATH,'/observed_points_optimizer'))
addpath(strcat(ROOT_PATH,'/monte_carlo_coverage_optimizer'))


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   User Options: Flags and Parameters                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set Parameters:
n_spacecraft = 4; % Number of Spacecraft, not counting the carrier

sc_obs_type  = [ones(1,n_spacecraft-1), 0]; % Index for instruments on board - 0 for carrier, 1 for not carrier

delta_t = 10*60; % [s]; simulation time step
total_t = 1*24*60*60; % [s]; 1 day, total time of simulation
time_vector = 0:delta_t:total_t;

n_trial_orbits = 10 ;
sc_max_memory = 8*20*1e9.*ones(1,n_spacecraft-1); % 20 GB max memory for instrument spacecraft 
sc_max_memory(1,n_spacecraft) = 8*10000*1e9; % Memory limit for carrier spacecraft 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                         Initialize Variables                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize SmallBodyParameters Struct
% Spherical harmonics model. Format is specified in the MATLAB function 'gravitysphericalharmonic.m'.
SphericalModel = {'EROS 433/Gravity_models/n15acoeff.tab', @readErosGravityModel};

% Load physical parameters of asteroid as a struct
SmallBodyParameters = get_Eros_body_parameters(SphericalModel{1}); % input is optional

% Name and path of shapefile. For now, only used for plotting.
SmallBodyParameters.shapefilename =  'EROS 433/MSI_optical_plate_models/eros022540.tab';

% Construct Small Body Parameters Struct
SmallBodyParameters.rotation_at_t0 = 0;
SmallBodyParameters.ShapeModel = get_shape_model(SmallBodyParameters.shapefilename) ; % Faces and verticies of Eros model
SmallBodyParameters.SphericalModel = SphericalModel;

%% Initialize Carrier Orbit
% Initialize carrier state
carrier_current_pos = [100000 0 0]; % [m]
v_magnitude = sqrt(SmallBodyParameters.Gravity.GM/norm(carrier_current_pos)); % [m/s]
v_direction = [0 1 0];
carrier_sc_velocity = v_magnitude*v_direction; % [m/s]
carrier_current_state = [carrier_current_pos carrier_sc_velocity];
ErosGravity = SphericalHarmonicsGravityIntegrator(SphericalModel,SmallBodyParameters.rotation_rate, 0, SmallBodyParameters.ShapeModel, @ode23tb);

% Integrate for carrier spacecraft
[~ ,carrier_orbit, ~] = ErosGravity.integrate(time_vector, carrier_current_state, 'absolute' ); % [m]

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                             Optimization                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Optimize Orbits and Observed Points with Monte Carlo - all but carrier
[map, observation_flow, observed_points, priority, reward, orbits] = monte_carlo_coverage_optmizer(SmallBodyParameters, time_vector, sc_obs_type, n_trial_orbits);

% Add in Carrier Orbit
orbits(:,:,end) = carrier_orbit;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              Show Results                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate Point Index Vector
point_index = zeros(1,size(map,2));
for i_sc = 1:n_spacecraft
    for i_v = 1:length(point_index)
        if map(i_sc,i_v)>0
            point_index(i_v) = i_sc; 
        end
    end
end

initialize_spatial_plot_3d()
render_asteroid_3d(SmallBodyParameters.ShapeModel);
render_observed_points_3d(SmallBodyParameters.ShapeModel.Vertices, point_index, n_spacecraft);
