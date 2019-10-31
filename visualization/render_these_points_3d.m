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

function [h] = render_these_points_3d(varargin)
%F_RENDER_OBSERVED_POINTS_3D Renders location of the observed points
%    Syntax: render_observed_points_3d(AsteroidModel, obs_points, 'color_array', *color_array, 'show_not_observed', *showNotObserved, *absolute, *plot_time)
%    *optional input
%
%   Inputs:
%    - AsteroidModel
%    - obs_points: Vector of indicies indicating the observed points on
%       the asteroid
%    - n_spacecraft: Number of spacecraft
%    - *color_array: Array of colors assigned to the spacecraft
%    - *showNoteObserved: if true, this will plot the points on the
%      asteroid that were not observed in black
%    - *absolute: if true, will plot the 3d points in an absolute frame,
%       rotating the body accordingly.
%    - *plot_time: if absolute is true, will plot the points at this point in
%    time
%
%   Outputs:
%    - h: plot handle for the points rendered

%% Interpret the Inputs
AsteroidModel = varargin{1}; % Asteroid Object
obs_points = varargin{2}; % Which points to render

if isempty(obs_points)
    
end

markersize = 3;
color_plot = ['c'];
showNotObserved = false;
absolute=false;
plot_time = NaN;
if nargin >= 3
    for i = 3:2:nargin
        if strcmpi(varargin{i},'color_array') || strcmpi(varargin{i},'colorArray') ||  strcmpi(varargin{i},'color')
            color_plot = varargin{i+1};
        end
        if strcmpi(varargin{i},'show_not_observed') || strcmpi(varargin{i},'showNotObserved')
            showNotObserved = varargin{i+1};
        end
        if strcmpi(varargin{i},'markersize') || strcmpi(varargin{i},'marker_size')
            markersize = varargin{i+1};
        end
        if strcmpi(varargin{i},'absolute')
            absolute = varargin{i+1};
        end
        if strcmpi(varargin{i},'plot_time')
            plot_time = varargin{i+1};
        end
    end
end
if absolute == true && isnan(plot_time)
    assert(false, "ERROR: if absolute plotting is selected, the variable 'plot_time' should be specified");
end

pos_points = AsteroidModel.BodyModel.shape.vertices ; % Convert to [km] for plotting
Nv = size(pos_points,1);
if absolute == true
    rot_angle_z = AsteroidModel.BodyModel.bodyFrame.pm.w0+ AsteroidModel.BodyModel.bodyFrame.pm.w * plot_time;
    Rot_i2b = rotmat(rot_angle_z,3);
    pos_points = (Rot_i2b'*pos_points')';
end

%% Render Points
h_notobserved = NaN;

if showNotObserved == true
    % Plot all of the points on the body which are not currently being observed
    not_observed_index = set_diff(1:Nv,obs_points);
    h_notobserved = plot3(pos_points(not_observed_index,1),pos_points(not_observed_index,2),...
        pos_points(not_observed_index,3),'ok','MarkerFaceColor','none','MarkerSize',markersize);
end

ns = 1;
observed_index = obs_points(obs_points~=0);
if sum(observed_index)>0
    h = plot3(pos_points(observed_index,1),pos_points(observed_index,2),...
        pos_points(observed_index,3),'o','MarkerFaceColor',color_plot,...
        'MarkerEdgeColor',color_plot,'MarkerSize',markersize);
    if ~isnan(h_notobserved)
        h = [h_notobserved, h];
    end
elseif ~isnan(h_notobserved)
    h = h_notobserved;
else
    % We did not plot anything
    h = plot3(0,0,0,'Color',[1 1 1,0.]);
end

end
