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

function neighbor_set = create_neighbor_set(ShapeModel)
%CREATE_NEIGHBOR_SET processes shape model data into a cell of connected
%nodes. Nodes are said to be connected if they share a face. 
%   Syntax: neighbor_set = create_neighbor_set(ShapeModel)
%   
%   Inputs: 
%    - ShapeModel: A struct with fields
%       - Verices [N_VERTICES x 3]: Vertex points of the body (in 3-space) 
%       - Faces [N_FACES x 3]: Faces defining body
% 
%   Outputs: 
%    - neighbor_set [N_VERTICES x 1]: A cell containing the neighbors (i.e.
%    connected verticies) for each vertex. 
%       e.g. neighbor_set{i} will contain all verticies sharing a face with
%       vertex i 

G = ShapeModel;
V = G.Vertices;
E = G.Faces;
n_vertices = size(V,1);
n_faces    = size(E,1);
vertex_set = 1:n_vertices;
N = cell(n_vertices, 1); % neighbor_set{i} returns neighbors of ith vertex
neighbor_set = cell(n_vertices, 1); 

for j = vertex_set
    for i = 1:n_faces
        
        if E(i,1)==j %|| E(i,2)==1 || E(i,3)==1
            if sum(N{j}==E(i,2))==0 % if not aleady in neighborset add
                N{j} = [N{j},E(i,2)];
            end
            if sum(N{j}==E(i,3))==0 % if not aleady in neighborset add
                N{j} = [N{j},E(i,3)];
            end
        end
        
        if E(i,2)==j %|| E(i,2)==1 || E(i,3)==1
            if sum(N{j}==E(i,1))==0 % if not aleady in neighborset add
                N{j} = [N{j},E(i,1)];
            end
            if sum(N{j}==E(i,3))==0 % if not aleady in neighborset add
                N{j} = [N{j},E(i,3)];
            end
        end
        
        if E(i,3)==j %|| E(i,2)==1 || E(i,3)==1
            if sum(N{j}==E(i,1))==0 % if not aleady in neighborset add
                N{j} = [N{j},E(i,1)];
            end
            if sum(N{j}==E(i,2))==0 % if not aleady in neighborset add
                N{j} = [N{j},E(i,2)];
            end
        end
        
    end
    
    neighbor_set{j} = sort(N{j}); 

end


end

