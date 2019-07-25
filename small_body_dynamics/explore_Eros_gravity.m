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
%         Usage example of the small body dynamics integrators.           %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = explore_Eros_gravity()

global SBDT_PATH
if ~exist("SBDT_PATH", 'var')
    SBDT_PATH = '/Users/frossi/Documents/JPL/ICC/SBDT';
end

testSBDT = false;
if exist(SBDT_PATH, 'dir')
    testSBDT = true;
else
    fprintf("WARNING: SBDT path not in %s.\n You should set SBDT folder as a global SBDT_PATH.\n SBDT tests will be skipped.\n", SBDT_PATH)
end
addpath(genpath('EROS 433'))

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%           In-house spherical harmonics integrator.                      %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Rotation rate. Rotation is assumed to be around the z axis (consistent
% with a spherical gravity model)
rotation_rate = 1639.389232 ... %degrees-day
                *pi/180 / 86400;

% Name and path of shapefile. For now, only used for plotting.
shapefilename = 'EROS 433/MSI_optical_plate_models/eros001708.tab';

ShapeModel.shapefilename = shapefilename;
ShapeModel.scale = 1; %Scale of shapefile. Trajectories are in m but are plotted in km.

% Spherical harmonics model. Format is specified in the MATLAB function
% 'gravitysphericalharmonic.m'.
SphericalModel = {'EROS 433/Gravity_models/n15acoeff.tab', @readErosGravityModel};

% Create an instance of the model we will use
ErosGravity = SphericalHarmonicsGravityIntegrator(SphericalModel,rotation_rate,0,ShapeModel,@ode23tb);

% Initialize a base case
sc_location = [35*1e3;0;0];
sc_radius = norm(sc_location);
GM = readErosGravityModel(SphericalModel{1});
sc_orbital_vel = sqrt(GM/sc_radius);
sc_vel = [0; sc_orbital_vel*sqrt(2)/2; sc_orbital_vel*sqrt(2)/2];
sc_state = [sc_location; sc_vel];
tspan = [0, 86400];

% Integrate
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

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%                           SBDT integrators.                             %
%                                                                         %
% The SBDT integrators are wrapped in a class that exposes a similar      %
% interface to the one used above. Initialization is different, but       %
% integration is performed with the same commands.                        %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if testSBDT
    userModelsPath = strcat(SBDT_PATH,'/ExampleUserModels');
    constantsModel = 1;
    addpath(strcat(SBDT_PATH,'/Startup'));
    global constants
    constants = addSBDT(SBDT_PATH, userModelsPath, constantsModel);

    eros_sbdt = loadEros( constants, 1, 1, 4, 3 );
    ErosGravity_SBDT = SphericalHarmonicsGravityIntegrator_SBDT(eros_sbdt);

    % Initialize a base case
    sc_location = [35*1e3;0;0];
    sc_radius = norm(sc_location);
    GM = eros_sbdt.gravity.gm * 1e9;  % Convert to km^3
    sc_orbital_vel = sqrt(GM/sc_radius);
    sc_vel = [0; sc_orbital_vel*sqrt(2)/2; sc_orbital_vel*sqrt(2)/2];
    sc_state = [sc_location; sc_vel];
    tspan = [0, 86400];

    [time,abs_traj,rel_traj] = ErosGravity_SBDT.integrate([0, 86400],sc_state,'absolute');

    AbsoluteTrajPlot= figure();
    RelativeTrajPlot=figure();
    ErosGravity_SBDT.plot_absolute_traj(time,abs_traj,0,AbsoluteTrajPlot,1)
    ErosGravity_SBDT.plot_relative_traj(time,rel_traj,1,RelativeTrajPlot,1)

    [time,abs_traj,rel_traj] = ErosGravity_SBDT.integrate([0, 86400],sc_state,'relative');

    AbsoluteTrajPlot= figure();
    RelativeTrajPlot=figure();
    ErosGravity_SBDT.plot_absolute_traj(time,abs_traj,0,AbsoluteTrajPlot,1)
    ErosGravity_SBDT.plot_relative_traj(time,rel_traj,1,RelativeTrajPlot,1)

    [time,abs_traj,state_transition_matrix] = ErosGravity_SBDT.integrate_absolute([0, 86400],sc_state);
else
    fprintf("WARNING: SBDT path not in %s.\n You should set SBDT folder as a global SBDT_PATH. \n Skipping SBDT tests.\n", SBDT_PATH)
end

end