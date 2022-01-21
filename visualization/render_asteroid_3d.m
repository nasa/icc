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

function [hdl] = render_asteroid_3d(Asteroid, absolute, plot_time, scale)
%F_RENDER_ASTEROID_3D Creates asteroid patch
%   Syntax: render_asteroid_3d(Asteroid, absolute, plot_time)
%   Inputs:
%   - Asteroid is a SBDT body model.
%   - absolute (optional, default=False) is a Boolean variable. If true, the asteroid is
%     plotted in an absolute reference frame.
%   - plot_time (optional, default=0) is a time. If absolute is false, plot_time is ignored.
%     if absolute is true, the asteroid is plotted at time plot_time.
%   - scale (optional,default=1) is a float. The specified scale is applied to the
%     plot.

% Create Asteroid Patch
if nargin<4
    scale=1;
end

if nargin<3
    plot_time=0;
end

if nargin<2
    absolute = false;
end


ShapeModel.Vertices = Asteroid.BodyModel.shape.vertices*scale;
ShapeModel.Faces = Asteroid.BodyModel.shape.faces;

if absolute == true
    assert(nargin>=3, "ERROR: if absolute plotting is desired, plot_time should be specified")
    rot_angle_z = Asteroid.BodyModel.bodyFrame.pm.w0+ Asteroid.BodyModel.bodyFrame.pm.w * plot_time;
    Rot_i2b = rotmat(rot_angle_z,3);
    ShapeModel.Vertices = (Rot_i2b'*ShapeModel.Vertices')';
end

hdl = patch(ShapeModel, 'FaceColor',0.7*[1 1 1], 'EdgeColor','none',...
    'FaceAlpha',1,'FaceLighting','flat','LineStyle','none',...
    'SpecularStrength',0,'AmbientStrength',.5);
material([0 1 0])
alpha(0.75)

hdl.EdgeColor = [.5, .5, .5];
hdl.LineStyle = '-';

end

