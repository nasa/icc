function [] = render_observed_points_2d(pos_points, point_index, n_spacecraft, above_or_below, color_array)
%RENDER_OBSERVED_POINTS_2D Summary of this function goes here
%   Detailed explanation goes here

if nargin < 5 
    color_array = ['c' 'r' 'b' 'g' 'm'];
end

hold on 

above_equator_index = logical(pos_points(:,3)>=0);
not_observed_index = logical(point_index == 0);

if strcmp(above_or_below, 'above')
    above_equator_not_observed_index = (above_equator_index & not_observed_index);
    
    plot(pos_points(above_equator_not_observed_index,1),pos_points(above_equator_not_observed_index,2),...
        '.k','MarkerFaceColor','none','MarkerSize',5)
    
    for i_sc = 1:1:n_spacecraft+1
        observed_index = logical(point_index == i_sc);
        above_equator_observed_index = (above_equator_index & observed_index);
        plot(pos_points(above_equator_observed_index,1),pos_points(above_equator_observed_index,2),'o','MarkerFaceColor',color_array(mod(i_sc,length(color_array))+1),'MarkerEdgeColor',color_array(mod(i_sc,length(color_array))+1),'MarkerSize',5)
    end
        
else
    above_equator_not_observed_index = (~above_equator_index & not_observed_index);
    
    plot(pos_points(above_equator_not_observed_index,1),pos_points(above_equator_not_observed_index,2),...
        '.k','MarkerFaceColor','none','MarkerSize',5)
    
    for i_sc = 1:1:n_spacecraft+1
        observed_index = logical(point_index == i_sc);
        above_equator_observed_index = (~above_equator_index & observed_index);
        plot(pos_points(above_equator_observed_index,1),pos_points(above_equator_observed_index,2),'o','MarkerFaceColor',color_array(mod(i_sc,length(color_array))+1),'MarkerEdgeColor',color_array(mod(i_sc,length(color_array))+1),'MarkerSize',5)
    end
    
end

end

