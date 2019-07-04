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
%        Usage example of Voronoi Computation on Small Body               %
%                                                                         %
% Demonstrates usage of the function get_voronoi_data(). Shows plots of   %
% the discrete Voronoi coverage regions and boundaries for a number of    %
% randomly generated "source" nodes.                                      %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear, clc, close all, run ../startup.m  % refresh

% Add Required Packages to PATH
addpath(genpath(strcat(ROOT_PATH,'/small_body_dynamics/EROS 433')))
addpath(strcat(ROOT_PATH,'/small_body_dynamics'))
addpath(strcat(ROOT_PATH,'/voronoi_coverage_optimizer')) % Add all utilities
addpath(strcat(ROOT_PATH,'/visualization'))

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   User Options: Flags and Parameters                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set Parameters:
n_sources = 6; % Number of generator points (or sources) on the body

% Flags
flag_demo = 1; % 1 for basic usage example; 2 for illustrative example of algorithm

% Set Plotting Preferences
color_array = ['r','g','b','c','m','y','k']; % colors used to index spacecraft

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

% Define a few variables for convienience
G = ErosShapeModel;
V = G.Vertices;
E = G.Faces;
n_vertices = size(V,1);

% Load the "neighbor_set" for Eros:
% neighbor_set{i} returns the (indicies of) verticies connected to vertex i
if isfile(strcat(ROOT_PATH,'/voronoi_coverage_optimizer/Eros_neighbor_set.mat'))
    load(strcat(ROOT_PATH,'/voronoi_coverage_optimizer/Eros_neighbor_set.mat'))
else
    neighbor_set = create_neighbor_set(ErosShapeModel);
    save(strcat(ROOT_PATH,'/voronoi_coverage_optimizer/Eros_neighbor_set.mat'),'neighbor_set')
end

% Initialize random sources
sources = cell(n_sources,1);
for i = 1:n_sources
    sources{i} = randi([1,n_vertices]) ;
end

if flag_demo == 1
    % Call the algorithm, show the results
    
    %% Calculate the Discrete Voronoi Data on the Mesh
    [voronoi_cells, voronoi_boundaries, voronoi_nodes] = get_voronoi_data(sources, neighbor_set);
    
    %% Plot Results
    figure();  hold on
    render_asteroid_3d(G);
    axis equal
    view(3)
    
    for i = 1:n_sources
        % Sources
        plot3(V(sources{i},1),V(sources{i},2),V(sources{i},3),'.','color', color_array(1+mod(i,6) ),'markersize', 35)
        
        % Boundaries
        plot3(V(voronoi_boundaries{i},1),V(voronoi_boundaries{i},2),V(voronoi_boundaries{i},3),'.','color', color_array(1+mod(i,6) ),'markersize', 14 )
        
        % Nodes
        plot3(V(voronoi_nodes{i},1),V(voronoi_nodes{i},2),V(voronoi_nodes{i},3),'k.','markersize', 20)
        
        % Coverage Areas
        plot3(V(voronoi_cells{i},1),V(voronoi_cells{i},2),V(voronoi_cells{i},3),'.','color', color_array(1+mod(i,6) ),'markersize', 4 )
    end
    
elseif flag_demo == 2
    % Iteratively call the algorithm with the wavefront propagation limited
    % at an increasing number of steps (demonstrational purposes only)
    
    for i_steps = 1:2:50
        %% Calculate the Discrete Voronoi Data on the Mesh
        [voronoi_cells, ~, ~] = get_voronoi_data(sources, neighbor_set, i_steps);
        
        %% Plot Results
        if i_steps ==1
            figure();  hold on
            render_asteroid_3d(G);
            axis equal
            view(3)
        else
            delete(coverage_points)
        end
        
        for i = 1:n_sources
            if i_steps == 1
                % Sources
                plot3(V(sources{i},1),V(sources{i},2),V(sources{i},3),'k.','markersize', 35)
            end
            
            % Coverage Areas
            coverage_points = plot3(V(voronoi_cells{i},1),V(voronoi_cells{i},2),V(voronoi_cells{i},3),'.','color', color_array(1+mod(i,6) ),'markersize', 8 );
        end
        drawnow limitrate
        pause(0.01)
        
    end
end