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

function [] = render_observed_points_2d(varargin)
%F_RENDER_OBSERVED_POINTS_2D Renders location of the observed points in 2d
%given Asteriod and Swarm objects 
%    Syntax: render_observed_points_2d(AsteroidModel, Swarm, above_or_below, 'color_array', *color_array, *font_size)
%    *optional input
%
%   Inputs:
%    - AsteroidModel
%    - Swarm
%    - above_or_below: {'above','below'} Side of the asteroid to be viewed
%       in the 2D projection
%    - *time_limits: the min and max time index to consider when plotting
%    - *color_array: Array of colors assigned to the spacecraft
%
%   Outputs:
%    - h: plot handle for the points rendered

%% Interpret the Inputs

AsteroidModel = varargin{1};
Swarm = varargin{2};

if nargin<3
    above_or_below = 'above';
else
    above_or_below = varargin{3};
end

timeLimitSpecified = false;

if nargin > 3
    for i = 3:1:nargin-1
        if strcmpi(varargin{i},'color_array') || strcmpi(varargin{i},'colorArray') ||  strcmpi(varargin{i},'color')
            color_array = varargin{i+1};
        end
        if strcmpi(varargin{i},'time_limits')
            timeLimitSpecified = true;
            time_limits = varargin{i+1};
        end
        if strcmpi(varargin{i},'font_size') || strcmpi(varargin{i},'fontsize') || strcmpi(varargin{i},'standard_font_size')
            standard_font_size = varargin{i+1};
        end
        if strcmpi(varargin{i},'title_font_size')
            title_font_size = varargin{i+1};
        end
        if strcmpi(varargin{i},'font_name')
            font_name = varargin{i+1};
        end
    end
end

if timeLimitSpecified == false
    time_limits = [1, length(Swarm.sample_times)];
end

pos_points = AsteroidModel.BodyModel.shape.vertices ; % Convert to [km] for plotting
n_spacecraft = Swarm.get_num_spacecraft();

point_index = zeros(1,size(pos_points,1));
for i_sc = 1:n_spacecraft
    obs_points = Swarm.Observation.observed_points(i_sc,time_limits(1):time_limits(end));
    obs_points(obs_points==0)= [];
    point_index(obs_points) = i_sc;
end

%% Plot

cla()
hold on
above_equator_index = transpose(logical(pos_points(:,3)>=0));
not_observed_index = logical(point_index == 0);

if strcmp(above_or_below, 'above')
    above_equator_not_observed_index = (above_equator_index & not_observed_index);
    
    plot(pos_points(above_equator_not_observed_index,1),pos_points(above_equator_not_observed_index,2),...
        '.k','MarkerFaceColor','none','MarkerSize',5)
    
    for i_sc = 1:1:n_spacecraft
        observed_index = logical(point_index == i_sc);
        above_equator_observed_index = (above_equator_index & observed_index);
        plot(pos_points(above_equator_observed_index,1),pos_points(above_equator_observed_index,2),'o','MarkerFaceColor',color_array(:,mod(Swarm.Parameters.types{i_sc},size(color_array,2))+1)','MarkerEdgeColor',color_array(:,mod(Swarm.Parameters.types{i_sc},size(color_array,2))+1)','MarkerSize',5)
    end
    
else
    above_equator_not_observed_index = (~above_equator_index & not_observed_index);
    
    plot(pos_points(above_equator_not_observed_index,1),pos_points(above_equator_not_observed_index,2),...
        '.k','MarkerFaceColor','none','MarkerSize',5)
    
    for i_sc = 1:1:n_spacecraft
        observed_index = logical(point_index == i_sc);
        above_equator_observed_index = (~above_equator_index & observed_index);
        plot(pos_points(above_equator_observed_index,1),pos_points(above_equator_observed_index,2),'o','MarkerFaceColor',color_array(:,mod(Swarm.Parameters.types{i_sc},size(color_array,2))+1)','MarkerEdgeColor',color_array(:,mod(Swarm.Parameters.types{i_sc},size(color_array,2))+1)','MarkerSize',5)
    end
    
end
title(strcat(upper(above_or_below(1)),above_or_below(2:end),' Equator'),'fontsize',title_font_size,'fontname',font_name)
xlabel('X axis [km]','fontsize',standard_font_size, 'fontname',font_name)
ylabel('Y axis [km]','fontsize',standard_font_size, 'fontname',font_name)
set(gca, 'fontsize',standard_font_size,'fontname',font_name)
axis equal


end

