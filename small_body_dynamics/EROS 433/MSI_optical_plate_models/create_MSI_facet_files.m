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
%          Create facet files from MSI data for SBDT use                  %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = create_MSI_facet_files(shape_model)
if nargin<1
    shape_model = 'eros001708';
end

vertices_file = strcat('V_', shape_model, '_vertices.txt');
facets_file   = strcat('F_', shape_model, '_facets.txt');

shapeModel = importdata(strcat(shape_model, '.tab'));
vertexIdx = strcmp(shapeModel.rowheaders, 'v');
facetIdx = strcmp(shapeModel.rowheaders, 'f');
assert(all(vertexIdx == 1-facetIdx));
vertices = shapeModel.data(vertexIdx,:);
facets = shapeModel.data(facetIdx,:);
% Move to one-based indexing
facets = facets + 1;
dlmwrite(facets_file,facets,' ');
dlmwrite(vertices_file,vertices,' ');