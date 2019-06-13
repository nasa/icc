function [h] = render_observed_points_3d(pos_points, point_index, num_SC, color_array, flag_showNotObserved )
%F_RENDER_OBSERVED_INDEX Summary of this function goes here
%   Detailed explanation goes here

if nargin == 4
    flag_showNotObserved = true;
elseif nargin == 3
    color_array = ['c' 'r' 'b' 'g' 'm'];
    flag_showNotObserved = true;
end

if flag_showNotObserved == true
    % Plot all of the points on the body which are not currently being observed
    not_observed_index = logical(point_index == 0);
    h(ns+1) = plot3(pos_points(not_observed_index,1),pos_points(not_observed_index,2),...
        pos_points(not_observed_index,3),'ok','MarkerFaceColor','none','MarkerSize',5);
end

% Plot points being observed, and colorcode by the agent number
for ns = 1:1:num_SC
    observed_index = logical(point_index == ns);
    h(ns) = plot3(pos_points(observed_index,1),pos_points(observed_index,2),...
        pos_points(observed_index,3),'o','MarkerFaceColor',color_array(mod(ns,length(color_array))+1),...
        'MarkerEdgeColor',color_array(mod(ns,length(color_array))+1),'MarkerSize',5);
end


end

