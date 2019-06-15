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
function [ShapeModel] = get_shape_model(shapefilename)
%F_GET_SHAPE_MODEL Generates a shape file from a *.tab file containing
%body parameters. 
%   Syntax: [ShapeModel] = get_shape_model(shapefilename)
%   
%   Inputs: 
%    - shapefilename: A string containing the directory of a *.tab file 
%       e.g. 'EROS 433/MSI_optical_plate_models/eros022540.tab'
%
%   Outputs: 
%    - ShapeModel: A struct containing the Faces and Verticies [m] of the
%       object described in shapefilename


% Eros shapefile. 
Raw_Model = importdata(shapefilename);
vertexIdx = strcmp(Raw_Model.rowheaders, 'v');
facetIdx = strcmp(Raw_Model.rowheaders, 'f');
assert(all(vertexIdx == 1-facetIdx));
Vertices = Raw_Model.data(vertexIdx,:);
Faces = Raw_Model.data(facetIdx,:);
Faces = Faces + 1;                 % Move to one-based indexing

% Create ShapeModel
ShapeModel.Vertices = Vertices.*10^3; % convert to [m]
ShapeModel.Faces = Faces;

end

