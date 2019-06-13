function [h] = initialize_spatial_plot_3d(standard_font_size)
%INITIALIZE_SPATIAL_PLOT_3D Summary of this function goes here
%   Detailed explanation goes here

if nargin < 1 
    standard_font_size = 25; 
end

hold on
xlabel('X axis [km]','fontsize',standard_font_size,'FontName','Times New Roman')
ylabel('Y axis [km]','fontsize',standard_font_size,'FontName','Times New Roman')
zlabel('Z axis [km]','fontsize',standard_font_size,'FontName','Times New Roman')
set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')


end

