function [h] = render_observed_points_3d(varargin)
%F_RENDER_OBSERVED_POINTS_3D Renders location of the observed points
%    Syntax: render_observed_points_3d(pos_points, point_index, n_spacecraft, 'color_array', *color_array, 'show_not_observed', *showNotObserved )
%    *optional input
%
%   Inputs:
%    - pos_points [km]: [N_VERTICES x 3] Array of model vertex points
%    - point_index [N_VERTICES x 1]: Vector of indicies indicating the
%       observed points on the asteroid.
%    - n_spacecraft: Number of spacecraft
%    - *color_array: Array of colors assigned to the spacecraft
%    - *showNotObserved: logical variable; will plot unobserved points
%       along with the observed points when set to true
%
%   Outputs:
%    - h: plot handle for the points rendered

%% Interpret the Inputs
n_inputs = max(size(varargin));
pos_points = varargin{1} ; % Convert to [km] for plotting
point_index = varargin{2} ;
n_spacecraft = varargin{3} ;

colorSpecified = false;
showNotObserved = false;

if n_inputs > 3
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
