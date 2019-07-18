function [h] = render_spacecraft_3d(varargin) 
%F_RENDER_SPACECRAFT_3D Renders location of the spacecraft in 3d space
%    Syntax: render_spacecraft_3d(sc_position_array, 'show_trail', *showTrail, 'color_array', *color_array)
%    *optional input
%
%   Inputs:
%    - sc_position_array [km]: [N_TIMESTEPS x 3 x N_SPACECRAFT] Position
%       array of the spacecraft (over some time history)
%    - *color_array: Array of colors assigned to the spacecraft
%    - *showTrail: logical variable; will show entire orbital
%
%   Outputs:
%    - h: plot handle for the points rendered

%% Interpret the Inputs
n_inputs = max(size(varargin));
sc_position_array = varargin{1}; % Convert to [km] for plotting 
n_spacecraft = size(sc_position_array,3);

colorSpecified = false;
showTrail = true;
if n_inputs > 1
    for i = 2:2:n_inputs
        if strcmpi(varargin{i},'color_array') || strcmpi(varargin{i},'colorArray') ||  strcmpi(varargin{i},'color')
            colorSpecified = true;
            color_array = varargin{i+1};
        end
        if strcmpi(varargin{i},'show_trail') || strcmpi(varargin{i},'showTrail')
            showTrail = varargin{i+1};
        end
    end
end

if colorSpecified == false
    color_array = ['c' 'r' 'b' 'g' 'm'];
end

%% Plot
hold on 
for i_sc = 1:n_spacecraft
    h(i_sc) = plot3( sc_position_array(end, 1, i_sc), sc_position_array(end, 2, i_sc), sc_position_array(end, 3, i_sc),...
        'o','MarkerFaceColor',color_array(mod(i_sc-1,length(color_array))+1), ...
        'MarkerEdgeColor',color_array(mod(i_sc-1,length(color_array))+1),'MarkerSize',10);
    
    if showTrail == true % shows the entire orbital path
        h(2*i_sc+1) = plot3( sc_position_array(:, 1, i_sc), sc_position_array(:, 2, i_sc), sc_position_array(:, 3, i_sc),...
            '-','Color',color_array(mod(i_sc-1,length(color_array))+1));
    end
end

end

