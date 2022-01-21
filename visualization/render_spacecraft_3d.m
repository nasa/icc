%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2019 by California Institute of Technology.  ALL RIGHTS RESERVED. %
% United  States  Government  sponsorship  acknowledged.   Any commercial use %
% must   be  negotiated  with  the  Office  of  Technology  Transfer  at  the %
% California Institute of Technology.                                         %
%                                                                             %
% This software may be subject to  U.S. export control laws  and regulations. %
% By accepting this document,  the user agrees to comply  with all applicable %
% U.S. export laws and regulations.  User  has the responsibility  to  obtain %
% export  licenses,  or  other  export  authority  as may be required  before %
% exporting  such  information  to  foreign  countries or providing access to %
% foreign persons.                                                            %
%                                                                             %
% This  software  is a copy  and  may not be current.  The latest  version is %
% maintained by and may be obtained from the Mobility  and  Robotics  Sytstem %
% Section (347) at the Jet  Propulsion  Laboratory.   Suggestions and patches %
% are welcome and should be sent to the software's maintainer.                %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [h] = render_spacecraft_3d(varargin) 
%F_RENDER_SPACECRAFT_3D Renders location of the spacecraft in 3d space
%    Syntax: render_spacecraft_3d(Swarm, time_index, show_trail*, color_array*, markersize*, linewidth*, absolute_frame_flag*)
%    *optional input
%
%   Inputs:
%    - *color_array: Array of colors assigned to the spacecraft
%    - *showTrail: logical variable; will show entire orbital
%
%   Outputs:
%    - h: plot handle for the points rendered

%% Interpret the Inputs
Swarm = varargin{1};
i_time = varargin{2};

markersize = 6; 
linewidth = 1; 
showTrail = true;
absolute= true;
plot_names = true;
color_array = ['r', 'b', 'g', 'c', 'm'];
standard_font_size = 25;

if length(varargin) > 2
    for i = 3:2:length(varargin)
        if strcmpi(varargin{i},'color_array') || strcmpi(varargin{i},'colorArray') ||  strcmpi(varargin{i},'color')
            color_array = varargin{i+1};
        end
        if strcmpi(varargin{i},'show_trail') || strcmpi(varargin{i},'showTrail')
            showTrail = varargin{i+1};
        end
        if strcmpi(varargin{i},'markersize') || strcmpi(varargin{i},'marker_size')
            markersize = varargin{i+1};
        end
        if strcmpi(varargin{i},'linewidth') || strcmpi(varargin{i},'line_width' )
            linewidth = varargin{i+1};
        end
        if strcmpi(varargin{i},'absolute') || strcmpi(varargin{i},'absolute_frame_flag')
            absolute = varargin{i+1};
        end

        if strcmpi(varargin{i},'plot_names') || strcmpi(varargin{i},'names')
            plot_names = varargin{i+1};
        end
        if strcmpi(varargin{i},'font_size') || strcmpi(varargin{i},'fontsize') || strcmpi(varargin{i},'standard_font_size')
            standard_font_size = varargin{i+1};
        end
    end
end

% Which trajectories do we use? This will help make the plotting code
% Convert to [km] for plotting 
if absolute == true
    sc_position_array = Swarm.abs_trajectory_array(1:i_time,:,:)/1e3;
else
    sc_position_array = Swarm.rel_trajectory_array(1:i_time,:,:)/1e3;
end
n_spacecraft = size(sc_position_array,3);

%% Plot
hold on
h = gobjects(3*n_spacecraft,1);
for i_sc = 1:n_spacecraft
    if markersize>0
        h(3*(i_sc-1)+2) = plot3( sc_position_array(end, 1, i_sc), sc_position_array(end, 2, i_sc), sc_position_array(end, 3, i_sc),...
            'o','MarkerFaceColor',color_array(:,mod(Swarm.Parameters.types{i_sc},size(color_array,2))+1)', ...
            'MarkerEdgeColor',color_array(:,mod(Swarm.Parameters.types{i_sc},size(color_array,2))+1)','MarkerSize',markersize);
    end
    if plot_names
%         h(3*(i_sc-1)+3) = text( sc_position_array(end, 1, i_sc), sc_position_array(end, 2, i_sc), sc_position_array(end, 3, i_sc),string(Swarm.Parameters.types{i_sc}));
        h(3*(i_sc-1)+3) = text( sc_position_array(end, 1, i_sc), sc_position_array(end, 2, i_sc), sc_position_array(end, 3, i_sc),string(i_sc), 'FontSize', standard_font_size);

    end
    if showTrail == true % shows the entire orbital path
        h(3*(i_sc-1)+1) = plot3( sc_position_array(:, 1, i_sc), sc_position_array(:, 2, i_sc), sc_position_array(:, 3, i_sc),...
            '-','Color',color_array(:,mod(Swarm.Parameters.types{i_sc},size(color_array,2))+1)','linewidth',linewidth);
    end
end

end

