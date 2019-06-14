function [handle] = render_asteroid_3d(ShapeModel)
%F_RENDER_ASTEROID_3D Creates asteroid patch
%   Syntax: render_asteroid_3d(ShapeModel)
%
%   Inputs:
%   - ShapeModel is a struct containing two fields:
%       Vertices: vertices of the asteroid
%       Faces: faces of the asteroid

% Create Asteroid Patch
handle = patch(ShapeModel, 'FaceColor',0.7*[1 1 1], 'EdgeColor','none',...
    'FaceAlpha',1,'FaceLighting','flat','LineStyle','none',...
    'SpecularStrength',0,'AmbientStrength',.5);
material([0 1 0])
alpha(0.75)

handle.EdgeColor = [.5, .5, .5];
handle.LineStyle = '-';

end

