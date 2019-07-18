function [h] = render_observed_points_3d(varargin)
%F_RENDER_OBSERVED_POINTS_3D Renders location of the observed points given
%the Asteroid and Swarm objects
%    Syntax: render_observed_points_3d(AsteroidModel, Swarm, 'color_array', *color_array, 'show_not_observed', *showNotObserved )
%    *optional input
%
%   Inputs:
%    - AsteroidModel
%    - Swarm
%
%   Outputs:
%    - h: plot handle for the points rendered

%% Interpret the Inputs
% n_inputs = max(size(varargin));
AsteroidModel = varargin{1}; 
Swarm = varargin{2}; 

pos_points = AsteroidModel.BodyModel.shape.vertices ; % Convert to [km] for plotting
n_spacecraft = Swarm.get_num_spacecraft(); 

point_index = zeros(1,size(pos_points,1));
for i_sc = 1:n_spacecraft
    obs_points = Swarm.Observation.observed_points(i_sc,:);
    obs_points(obs_points==0)= [];
    point_index(obs_points) = i_sc; 
end

colorSpecified = false;
showNotObserved = false;

if nargin > 3
    for i = 4:2:n_inputs
        if strcmpi(varargin{i},'color_array') || strcmpi(varargin{i},'colorArray') ||  strcmpi(varargin{i},'color')
            colorSpecified = true;
            color_array = varargin{i+1};
        end
        if strcmpi(varargin{i},'show_not_observed') || strcmpi(varargin{i},'showNotObserved')
            showNotObserved = varargin{i+1};
        end
    end
end

if colorSpecified == false
    color_array = ['c' 'r' 'b' 'g' 'm'];
end

%% Render Points

if showNotObserved == true
    % Plot all of the points on the body which are not currently being observed
    not_observed_index = logical(point_index == 0);
    h(ns+1) = plot3(pos_points(not_observed_index,1),pos_points(not_observed_index,2),...
        pos_points(not_observed_index,3),'ok','MarkerFaceColor','none','MarkerSize',5);
end

% Plot points being observed, and colorcode by the agent number
for ns = 1:1:n_spacecraft
    observed_index = logical(point_index == ns);
    if sum(observed_index)>0
        h(ns) = plot3(pos_points(observed_index,1),pos_points(observed_index,2),...
            pos_points(observed_index,3),'o','MarkerFaceColor',color_array(mod(ns,length(color_array))+1),...
            'MarkerEdgeColor',color_array(mod(ns,length(color_array))+1),'MarkerSize',5);
    end
end

end
