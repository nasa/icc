function [h] = render_these_points_3d(varargin)
%F_RENDER_OBSERVED_POINTS_3D Renders location of the observed points
%    Syntax: render_observed_points_3d(AsteroidModel, obs_points, 'color_array', *color_array, 'show_not_observed', *showNotObserved )
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
%
%   Outputs:
%    - h: plot handle for the points rendered

%% Interpret the Inputs
AsteroidModel = varargin{1}; % Asteroid Object
obs_points = varargin{2}; % Which points to render
pos_points = AsteroidModel.BodyModel.shape.vertices ; % Convert to [km] for plotting
Nv = size(pos_points,1);
markersize = 3;
colorSpecified = false;
showNotObserved = false;
if nargin >= 3
    for i = 3:2:nargin
        if strcmpi(varargin{i},'color_array') || strcmpi(varargin{i},'colorArray') ||  strcmpi(varargin{i},'color')
            colorSpecified = true;
            color_array = varargin{i+1};
        end
        if strcmpi(varargin{i},'show_not_observed') || strcmpi(varargin{i},'showNotObserved')
            showNotObserved = varargin{i+1};
        end
        if strcmpi(varargin{i},'markersize') || strcmpi(varargin{i},'marker_size')
            markersize = varargin{i+1};
        end
    end
end

if colorSpecified == false
    color_array = ['c' 'r' 'b' 'g' 'm'];
end

%% Render Points

if showNotObserved == true
    % Plot all of the points on the body which are not currently being observed
    not_observed_index = set_diff(1:Nv,obs_points);
    h(ns+1) = plot3(pos_points(not_observed_index,1),pos_points(not_observed_index,2),...
        pos_points(not_observed_index,3),'ok','MarkerFaceColor','none','MarkerSize',markersize);
end

ns = 1;
observed_index = obs_points;
if sum(observed_index)>0
    h(ns) = plot3(pos_points(observed_index,1),pos_points(observed_index,2),...
        pos_points(observed_index,3),'o','MarkerFaceColor',color_array(mod(ns,length(color_array))+1),...
        'MarkerEdgeColor',color_array(mod(ns,length(color_array))+1),'MarkerSize',markersize);
end

end
