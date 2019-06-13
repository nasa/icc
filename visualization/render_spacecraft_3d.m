function [h] = render_spacecraft_3d(sc_position_array, flag_renderOrbit, color_array)
%F_RENDER_SPACECRAFT_3D show spacecraft location and orbital trajectory
%   Detailed explanation goes here

if nargin < 3
    color_array = ['c' 'r' 'b' 'g' 'm'];
end
if nargin < 2
    flag_renderOrbit = true;
end

% Infer number of spacecraft
nSpacecraft = size(sc_position_array,3) ;

% Plot
for i_sc = 1:nSpacecraft
    h(i_sc) = plot3( sc_position_array(end, 1, i_sc), sc_position_array(end, 2, i_sc), sc_position_array(end, 3, i_sc),...
        'o','MarkerFaceColor',color_array(mod(i_sc,length(color_array))+1), ...
        'MarkerEdgeColor',color_array(mod(i_sc,length(color_array))+1),'MarkerSize',10);
    
    if flag_renderOrbit == true % shows the entire orbital path
        h(2*i_sc+1) = plot3( sc_position_array(:, 1, i_sc), sc_position_array(:, 2, i_sc), sc_position_array(:, 3, i_sc),...
            '-','Color',color_array(mod(i_sc,length(color_array))+1));
    end
end

end

