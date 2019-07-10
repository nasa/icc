%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

function [voronoi_cells, voronoi_boundaries, voronoi_nodes] = get_voronoi_data(varargin)
%GET_VORONOI_DATA computes information for a discrete Voronoi tesselation
%from a given set of sources using wavefront propagation. 
%Note: the current implemenation does not account for the Euclidian
%distance between nodes. 
%   [voronoi_cells, voronoi_boundaries, voronoi_nodes] = get_voronoi_data(sources, neighbor_set, *max_steps)
%   *optional input
%   
%   Inputs: 
%    - sources [N_SOURCES x 1]: The indicies for a set of source nodes on the small body 
%    - neighbor_set [N_VERTICES x 1]: A cell containing the neighbors (i.e.
%    connected verticies) for each vertex. 
%       e.g. neighbor_set{i} will contain all verticies sharing a face with
%       vertex i 
% 
%   Outputs: 
%    - voronoi_cells [N_SOURCES x 1]: A cell containing the nodes in the
%    coverage region for each Voronoi tesselation
%       e.g. voronoi_cells{i} is a vector containing the indicies of all of 
%       the points in the ith partition of the tesselation 
%    - voronoi_boundaries [N_SOURCES x 1]: A cell containing the nodes
%    composing the boundary of each of the repective partitions 
%       e.g. voronoi_boundaries{i} is a vector containing the indicies of
%       all of the nodes in the ith partition that have neighbors in a 
%       different cell
%    - voronoi_nodes [N_SOURCES x 1]: A cell containing the points in each 
%    partition have neighboring nodes in at least two other cells 

sources = varargin{1}; 
neighbor_set = varargin{2};
if nargin < 3 
    max_steps = inf; % maximum iterations in wave propagation 
else
    max_steps = varargin{3};
end

%% Initialize Variables 
n_sources = length(sources);
voronoi_cells = sources; % Voronoi cells start at sources and expand until collision with other expanding sources

boundary_set = cell(n_sources,1); % The outside border of the expanding cell. This is removed when another cell is encoutered. 
voronoi_boundaries =  cell(n_sources,1); % These are created from the colliding wavefronts 
voronoi_nodes =  cell(n_sources,1); % Nodes are points on the Voronoi boundaries neighboring at least two other boundaries 
for i = 1:n_sources
    boundary_set{i} = neighbor_set{voronoi_cells{i}};
    voronoi_boundaries{i} = [];
    voronoi_nodes{i} = [];
end

bordering_nodes = []; % nodes "colliding" with other boundaries 

%% Step 1: Calculate Voronoi Cells and Boundary
j = 0; 
continue_search = true; 
while (j<max_steps)&&(continue_search==true)
    j=j+1; 
    for i=1:n_sources 
        % Get candidate explored nodes
        [c_explored_vertices, c_boundary_set] = propagate_wavefront(voronoi_cells{i}, boundary_set{i}, neighbor_set, 1);
        
        % Move any points that cross into another boundary to the Voronoi boundary set
        i_remove = [];
        for k=1:n_sources % check that i is not in k
            if k~=j
                for p = 1:length(c_boundary_set)
                    if sum(c_boundary_set(p)==boundary_set{k})>0 || sum(c_boundary_set(p)==voronoi_cells{k})>0
                        i_remove = [i_remove, p] ;
                    end
                end
            end
        end
        bordering_nodes = unique([bordering_nodes, c_boundary_set(i_remove)]);
        
        c_boundary_set(i_remove) = [] ;
        boundary_set{i} = c_boundary_set;
        voronoi_cells{i} = c_explored_vertices;
    end

    % Break loop once completely covered
    n_empty_bounds = 0;
    for i = 1:n_sources
        if isempty(boundary_set{i})
            n_empty_bounds = n_empty_bounds + 1;
        end
    end
    if n_empty_bounds==n_sources
        continue_search = false; 
    end
end

% Create Voronoi Boundaries
for i=1:n_sources
    voronoi_boundaries{i} = intersect(voronoi_cells{i}, bordering_nodes); % voronoi_boundaries{i} = coverage points in cell i lying on edge of cell
end

%% Step 2: Calculate Voronoi Nodes
% Voronoi nodes are boundary_points neighboring two or more other boundaries
if n_sources>2
    for i=1:n_sources
        for j = 1:n_sources
            if j~=i
                % Isolate the points in boundary i which border boundary j
                ij_neighbors = intersect(voronoi_boundaries{i}, get_node_neighbors(voronoi_boundaries{j},neighbor_set));
                
                if ~isempty(ij_neighbors) % if i and j are adjacent boundaries
                    % isolate the points in boundary k which borders the ij boundary
                    for k = 1:n_sources
                        if k~=j && k~=i
                            ijk_neighbors = intersect(ij_neighbors, get_node_neighbors(voronoi_boundaries{k},neighbor_set));
                            if ~isempty(ijk_neighbors)
                                for p = 1:length(ijk_neighbors)
                                    voronoi_nodes{i} = [voronoi_nodes{i}, ijk_neighbors(1)];
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end

end



