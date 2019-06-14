function [] = initialize_spatial_plot_3d(varargin)
%INITIALIZE_SPATIAL_PLOT_3D Initializes 3D spatial plot with axes in km
%   Syntax: [] = initialize_spatial_plot_3d('font_size', font_size, 'limits', axes_limits)
%    *all inputs are optional
%
%   Inputs:
%    - font_size: Font size used for axes labels
%    - axes_limits: [km] Limits used on axes; axes lengths are equal

%% Interpret the Inputs
n_inputs = max(size(varargin));
limitsSpecified = false;
fontSizeSpecified = false;
for i = 1:2:n_inputs
    if strcmpi(varargin{i},'font_size') || strcmpi(varargin{i},'fontsize')
        fontSizeSpecified = true;
        standard_font_size = varargin{i+1};
    end
    if strcmpi(varargin{i},'limits')
        limitsSpecified = true;
        axes_limits = varargin{i+1};
    end
end

%% Make Plot
if fontSizeSpecified==false
    standard_font_size = 25;
end

hold on
xlabel('X axis [km]','fontsize',standard_font_size,'FontName','Times New Roman')
ylabel('Y axis [km]','fontsize',standard_font_size,'FontName','Times New Roman')
zlabel('Z axis [km]','fontsize',standard_font_size,'FontName','Times New Roman')
set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')

grid minor;
view(3)

if limitsSpecified==true
    xlim(axes_limits); ylim(axes_limits); zlim(axes_limits);
end
hold on

end

