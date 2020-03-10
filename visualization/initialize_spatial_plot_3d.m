function [] = initialize_spatial_plot_3d(varargin)
%INITIALIZE_SPATIAL_PLOT_3D Initializes 3D spatial plot with axes in km
%   Syntax: [] = initialize_spatial_plot_3d(font_size*, axes_limits*, font_name*)
%    *all inputs are optional
%
%   Inputs:
%    - font_size: Font size used for axes labels
%    - axes_limits: [km] Limits used on axes; axes lengths are equal

%% Interpret the Inputs
limitsSpecified = false;
axes_limits = [-1 1 -1 1 -1 1].*40;
standard_font_size = 25;
font_name = 'Times New Roman';

for i = 1:2:length(varargin)
    if strcmpi(varargin{i},'font_size') || strcmpi(varargin{i},'fontsize') || strcmpi(varargin{i},'standard_font_size')
        standard_font_size = varargin{i+1};
    end
    if strcmpi(varargin{i},'limits')
        limitsSpecified = true;
        axes_limits = varargin{i+1}; 
        assert(length(axes_limits)==6, "ERROR: axes limits size is incorrect")
    end
    if strcmpi(varargin{i},'font_name')
        font_name = varargin{i+1};
    end
end

%% Make Plot

hold on
xlabel('X axis [km]','fontsize',standard_font_size, 'fontname',font_name)
ylabel('Y axis [km]','fontsize',standard_font_size, 'fontname',font_name)
zlabel('Z axis [km]','fontsize',standard_font_size, 'fontname',font_name)
set(gca, 'fontsize',standard_font_size, 'fontname',font_name)

grid on;
view(3)

if limitsSpecified==true
    xlim(axes_limits); ylim(axes_limits); zlim(axes_limits);
end
hold on

end

