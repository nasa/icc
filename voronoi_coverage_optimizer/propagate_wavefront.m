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

function [explored_vertices, boundary_set] = propagate_wavefront(varargin)
%PROPAGATE_WAVEFRONT propagates a "wavefront" of explored nodes over the 
%small body. i.e. updates the explored verticies and boundary by exploring 
%additional neighbors within a distance of "n_steps". 
%Note that this function can be used recursively. 
%   Syntax:  [explored_vertices, boundary_set] = propagate_wavefront(explored_verticies, boundary_set, neighbor_set, *n_steps)
%   *optional input 
%   
%   Inputs: 
%    - explored_verticies: A vector of explored nodes  
%    - boundary_set: A vector of the nodes on the boundary of the explored
%    region 
%    - neighbor_set [N_VERTICES x 1]: A cell containing the neighbors (i.e.
%    connected verticies) for each vertex. 
%       e.g. neighbor_set{i} will contain all verticies sharing a face with
%       vertex i 
%    - *n_steps: Number of iterations in the expansion. Each iteration will
%    move the boundary of explored points all neighboring verticies. 

% 
%   Outputs: 
%    - explored_verticies: An updated vector of explored nodes  
%    - boundary_set: An updated vector of the nodes on the boundary of the
%    explored region 

if nargin<4
    n_steps = 1;
else
    n_steps = varargin{4};
end

explored_vertices = varargin{1} ;
boundary_set = varargin{2} ;
neighbor_set = varargin{3} ;

for i = 1:n_steps
    explored_vertices = sort([explored_vertices, boundary_set]); % add current boundary to expored set.
    boundary_set = get_node_neighbors(boundary_set, neighbor_set );
    
    remove_indices = [];
    
    for j = 1:length(boundary_set)
        if sum(boundary_set(j)==explored_vertices)~=0
            remove_indices = [remove_indices, j];
        end
    end
    boundary_set(remove_indices)=[]; % remove from bounday if it is in the explored region
   
end

end