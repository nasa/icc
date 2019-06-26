function [explored_vertices, boundary_set] = propagate_wavefront( varargin )
%PROPAGATE_WAVEFRONT given a set of explored verticies
%   Detailed explanation goes here

if nargin<4
    n_steps = 1;
else
    n_steps = varargin{4};
end

explored_vertices = varargin{1} ;
boundary_set = varargin{2} ;
neighbor_set = varargin{3} ;

for i = 1:n_steps
    %     old_explored_verticies = explored_vertices;
    explored_vertices = sort([explored_vertices, boundary_set]); % add current boundary to expored set.
    boundary_set = get_node_neighbors(boundary_set, neighbor_set );
    
    % new boundary is old boundaries neighbors minus explored - remove explored
    remove_indices = [];
    
    for j = 1:length(boundary_set)
        if sum(boundary_set(j)==explored_vertices)~=0
            remove_indices = [remove_indices, j];
        end
    end
    boundary_set(remove_indices)=[]; % remove from bounday if it is in the explored region
   
end


end