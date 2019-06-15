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
%         Usage example of the small body dynamics integrators.           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear, clc, close all, run ../startup.m  % refresh 

% Add Required Packages to PATH 
addpath(genpath(strcat(ROOT_PATH,'/small_body_dynamics/EROS 433')))
addpath(strcat(ROOT_PATH,'/small_body_dynamics'))

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   User Options: Flags and Parameters                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

delta_t = 10*60; % [sec] simulation time step
total_t = 5*24*60*60; % [sec]; 5 days, total time of simulation

flag_testInHouse = true; % test in-house spherical harmonics integrator
flag_testSBDT = false; % test SBDT integrators 

tspan = [0: delta_t: total_t];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%           In-house spherical harmonics integrator.                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag_testInHouse
    %% Set up Environment 
    
    % Spherical harmonics model. Format is specified in the MATLAB function 'gravitysphericalharmonic.m'.
    SphericalModel = {strcat(ROOT_PATH,'/small_body_dynamics/EROS 433/Gravity_models/n15acoeff.tab'), @readErosGravityModel};
    
    % Load physical parameters of asteroid as a struct
    ErosParameters = get_Eros_body_parameters(SphericalModel{1}); 
    
    % Name and path of shapefile. For now, only used for plotting.
    shapefilename = strcat(ROOT_PATH,'/small_body_dynamics/EROS 433/MSI_optical_plate_models/eros001708.tab');

    ErosShapeModel.shapefilename = shapefilename;
    ErosShapeModel.scale = 1; %Scale of shapefile. Trajectories are in m but are plotted in km.
    
    % Create an instance of the model we will use
    ErosGravity = SphericalHarmonicsGravityIntegrator(SphericalModel,ErosParameters.rotation_rate,0,ErosShapeModel,@ode23tb);

    %% Integrate Trajectory
    
    % Initialize a base case
    sc_position = [35*1e3;0;0];
    sc_radius = norm(sc_position);
    sc_orbital_vel = sqrt(ErosParameters.Gravity.GM/sc_radius);
    sc_vel = [0; sc_orbital_vel*sqrt(2)/2; sc_orbital_vel*sqrt(2)/2];
    sc_state = [sc_position; sc_vel];
    
    [time,abs_traj,rel_traj] = ErosGravity.integrate(tspan,sc_state,'absolute');
    AbsoluteTrajPlot= figure();
    RelativeTrajPlot=figure();
    ErosGravity.plot_absolute_traj(time,abs_traj,1,AbsoluteTrajPlot,1)
    ErosGravity.plot_relative_traj(time,rel_traj,1,RelativeTrajPlot,1)

    [time,abs_traj,rel_traj] = ErosGravity.integrate(tspan,sc_state,'relative');
    AbsoluteTrajPlot= figure();
    RelativeTrajPlot=figure();
    ErosGravity.plot_absolute_traj(time,abs_traj,1,AbsoluteTrajPlot,1)
    ErosGravity.plot_relative_traj(time,rel_traj,1,RelativeTrajPlot,1)
end 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           SBDT integrators.                             %
%                                                                         %
% The SBDT integrators are wrapped in a class that exposes a similar      %
% interface to the one used above. Initialization is different, but       %
% integration is performed with the same commands.                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if flag_testSBDT
    userModelsPath = strcat(SBDT_PATH,'/ExampleUserModels');
    constantsModel = 1;
    addpath(strcat(SBDT_PATH,'/Startup'));
    global constants
    constants = addSBDT(SBDT_PATH, userModelsPath, constantsModel);
    
    % In order to select a different gravity model, change the inputs to
    % the loadEros function. See the help for assistance
    eros_sbdt = loadEros( constants, 1, 1, 4, 3 );
    ErosGravity_SBDT = SphericalHarmonicsGravityIntegrator_SBDT(eros_sbdt);

    % Initialize a base case
    sc_position = [35*1e3;0;0];
    sc_radius = norm(sc_position);
    GM = eros_sbdt.gravity.gm * 1e9;  % Convert to km^3
    sc_orbital_vel = sqrt(GM/sc_radius);
    sc_vel = [0; sc_orbital_vel*sqrt(2)/2; sc_orbital_vel*sqrt(2)/2];
    sc_state = [sc_position; sc_vel];

    [time,abs_traj,rel_traj] = ErosGravity_SBDT.integrate(tspan,sc_state,'absolute');

    AbsoluteTrajPlot= figure();
    RelativeTrajPlot=figure();
    ErosGravity_SBDT.plot_absolute_traj(time,abs_traj,0,AbsoluteTrajPlot,1)
    ErosGravity_SBDT.plot_relative_traj(time,rel_traj,1,RelativeTrajPlot,1)

    [time,abs_traj,rel_traj] = ErosGravity_SBDT.integrate(tspan,sc_state,'relative');

    AbsoluteTrajPlot= figure();
    RelativeTrajPlot=figure();
    ErosGravity_SBDT.plot_absolute_traj(time,abs_traj,0,AbsoluteTrajPlot,1)
    ErosGravity_SBDT.plot_relative_traj(time,rel_traj,1,RelativeTrajPlot,1)

    [time,abs_traj,state_transition_matrix] = ErosGravity_SBDT.integrate_absolute(tspan,sc_state);
else
    fprintf("WARNING: SBDT path not in %s.\n You should set SBDT folder as a global SBDT_PATH. \n Skipping SBDT tests.\n", SBDT_PATH)
end


