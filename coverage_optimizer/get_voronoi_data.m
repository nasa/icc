function [voronoi_cells, voronoi_boundaries, voronoi_nodes] = get_voronoi_data(varargin)

sources = varargin{1}; 
neighbor_set = varargin{2};
if nargin < 3 
    max_steps = 500; % maximum iterations in wave propagation 
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

bordering_nodes = []; % candidate locations for the voronoi boundary

%% Step 1: Calculate Voronoi Cells and Boundary

for j = 1:max_steps % max_steps is search stop limit
    for i=1:n_sources % Propagate source i
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
        break
    end
end

% Create: Voronoi boundaries
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



