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
%               Usage example of Observed Points Optimizer                %
%                                                                         %
% Demonstrates usage of the observed_points_optimizer module by finding   %
% the observation points accociated with randomly generated orbits.       %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear, clc, close all, run ../startup.m  % refresh

% Add Required Packages to PATH
addpath(genpath(strcat(ROOT_PATH,'/small_body_dynamics/EROS 433')))
addpath(strcat(ROOT_PATH,'/small_body_dynamics'))
addpath(genpath(strcat(ROOT_PATH,'/utilities'))) % Add all utilities 
addpath(strcat(ROOT_PATH,'/visualization'))
addpath(strcat(ROOT_PATH,'/observed_points_optimizer'))

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   User Options: Flags and Parameters                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set Parameters:
n_spacecraft = 2; % Number of Spacecraft, not counting the carrier 
sc_obs_type  = ones(1,n_spacecraft); % Index for instruments on board - 0 for carrier, 1 for not carrier 
delta_t = 10*60; % [s]; simulation time step
total_t = 1*24*60*60; % [s]; 1 day, total time of simulation

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Initialize Simulation Environment Variables                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Spherical harmonics model. Format is specified in the MATLAB function 'gravitysphericalharmonic.m'.
SphericalModel = {'EROS 433/Gravity_models/n15acoeff.tab', @readErosGravityModel};

% Load physical parameters of asteroid as a struct
ErosParameters = get_Eros_body_parameters(SphericalModel{1}); % input is optional

% Name and path of shapefile. For now, only used for plotting.
shapefilename = 'EROS 433/MSI_optical_plate_models/eros022540.tab';

ErosShapeModel = get_shape_model(shapefilename) ; % Faces and verticies of Eros model
pos_points = ErosShapeModel.Vertices;
num_points = size(pos_points,1);

% Create an instance of the dynamics model we will use
ErosGravity = SphericalHarmonicsGravityIntegrator(SphericalModel,ErosParameters.rotation_rate, 0, ErosShapeModel, @ode23tb);

% Initialize point index. This keeps track of the observed points on Eros.
point_index = zeros(num_points,1);

% Initialize memory use array. This keeps track of the memory usage for the spacecraft
sc_memory_use = zeros(n_spacecraft,1);

% Construct Small Body Parameters Struct 
SmallBodyParameters = ErosParameters; 
SmallBodyParameters.rotation_at_t0 = 0; 
SmallBodyParameters.ShapeModel = ErosShapeModel; 


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          Integrate Trajectory                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize other spacecraft states (i.e. swarm state)
sc_initial_state_array = initialize_random_orbits(n_spacecraft, ErosParameters); 

time_vector = 0:delta_t:total_t;

% Integrate for instrument spacecraft
orbits = zeros( length(time_vector), 6, n_spacecraft);
for i_sc = 1:n_spacecraft
    [~, abs_traj, ~] = ErosGravity.integrate(time_vector, sc_initial_state_array(i_sc, :), 'absolute' );
    orbits(:, :, i_sc) = abs_traj;
end

% Instantiate SwarmTrajectory class for handling orbit data
swarm   = SwarmTrajectory(orbits, time_vector); % instrument and relay spacecraft

% Check for collisions
if swarm.collision_with_asteroid(ErosParameters.radius)
    fprintf('\nCollision with the asteroid!\n\n')
    percentage_seen = 0;
    return
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Find Best Observation Points for Given Orbit                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[map, observation_flow, observed_points, priority, reward] = observed_points_optimizer_main(SmallBodyParameters, time_vector, sc_obs_type, orbits); 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 Show Observed Points Map with Orbits                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate Point Index Vector 
point_index = zeros(1,size(map,2));
for i_sc = 1:n_spacecraft
    point_index = point_index + (map(i_sc,:)>0).*i_sc;
end

% Show Observed Points 
initialize_spatial_plot_3d()
render_asteroid_3d(SmallBodyParameters.ShapeModel);
render_observed_points_3d(SmallBodyParameters.ShapeModel.Vertices, point_index, n_spacecraft);
