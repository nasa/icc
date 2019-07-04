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

function unique_new_neighbors = get_node_neighbors(vertices , neighbor_set)
%GET_NODE_NEIGHBORS returns the set all neighboring vertices to a given set
%of vertices. Removes repreated elements. 
%   Syntax: unique_new_neighbors = get_node_neighbors(vertices , neighbor_set)
%   
%   Inputs: 
%    - verticies [N_VERTICES x 1]: The indicies for a set of verticies  
%    - neighbor_set [N_VERTICES x 1]: A cell containing the neighbors (i.e.
%    connected verticies) for each vertex. 
%       e.g. neighbor_set{i} will contain all verticies sharing a face with
%       vertex i 
% 
%   Outputs: 
%    - unique_new_neighbors: A vector of all neighboring vertices to the
%    given input set, with repeated elements removed.

sort = false; 

new_neighbors = []; % neighbors of first vertex

for i = 1:length(vertices)
    new_neighbors = [new_neighbors, neighbor_set{vertices(i)}];
end
if sort==true
    unique_new_neighbors = sort(unique(new_neighbors));
else
    unique_new_neighbors = unique(new_neighbors);
end

end


