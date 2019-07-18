function [handle] = render_asteroid_3d(Asteroid)
%F_RENDER_ASTEROID_3D Creates asteroid patch
%   Syntax: render_asteroid_3d(ShapeModel)
%
%   Inputs:
%   - ShapeModel is a struct containing two fields:
%       Vertices: vertices of the asteroid [m]
%       Faces: faces of the asteroid

% Create Asteroid Patch
ShapeModel.Vertices = Asteroid.BodyModel.shape.vertices;
ShapeModel.Faces = Asteroid.BodyModel.shape.faces;

handle = patch(ShapeModel, 'FaceColor',0.7*[1 1 1], 'EdgeColor','none',...
    'FaceAlpha',1,'FaceLighting','flat','LineStyle','none',...
    'SpecularStrength',0,'AmbientStrength',.5);
material([0 1 0])
alpha(0.75)

handle.EdgeColor = [.5, .5, .5];
handle.LineStyle = '-';

end

