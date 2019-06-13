function [Eros_Shape_Model] = get_shape_model(shapefilename)
%F_GET_SHAPE_MODEL Summary of this function goes here
%   Detailed explanation goes here

% Eros shapefile. For now, only used for plotting.
% shapefilename = 'EROS 433/MSI_optical_plate_models/eros001708.tab';
Raw_Model = importdata(shapefilename);
vertexIdx = strcmp(Raw_Model.rowheaders, 'v');
facetIdx = strcmp(Raw_Model.rowheaders, 'f');
assert(all(vertexIdx == 1-facetIdx));
Vertices = Raw_Model.data(vertexIdx,:);
Faces = Raw_Model.data(facetIdx,:);
Faces = Faces + 1;                 % Move to one-based indexing
Eros_Shape_Model.Vertices = Vertices;
Eros_Shape_Model.Faces = Faces;

end

