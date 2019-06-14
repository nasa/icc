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

