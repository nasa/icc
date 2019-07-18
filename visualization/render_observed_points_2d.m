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
%    Syntax: render_observed_points_2d(AsteroidModel, Swarm, above_or_below, 'color_array', *color_array)
%    *optional input
%
%   Inputs:
%    - AsteroidModel
%    - Swarm
%    - above_or_below: {'above','below'} Side of the asteroid to be viewed
%       in the 2D projection
%    - *color_array: Array of colors assigned to the spacecraft
%
%   Outputs:
%    - h: plot handle for the points rendered

%% Interpret the Inputs

AsteroidModel = varargin{1};
Swarm = varargin{2};

pos_points = AsteroidModel.BodyModel.shape.vertices ; % Convert to [km] for plotting
n_spacecraft = Swarm.get_num_spacecraft();

point_index = zeros(1,size(pos_points,1));
for i_sc = 1:n_spacecraft
    obs_points = Swarm.Observation.observed_points(i_sc,:);
    obs_points(obs_points==0)= [];
    point_index(obs_points) = i_sc;
end

if nargin<3
    above_or_below = 'above';
else
    above_or_below = varargin{3};
end

colorSpecified = false;

if nargin > 3
    if strcmpi(varargin{5},'color_array') || strcmpi(varargin{5},'colorArray') ||  strcmpi(varargin{5},'color')
        colorSpecified = true;
        color_array = varargin{6};
    end
end

if colorSpecified == false
    color_array = ['c' 'r' 'b' 'g' 'm'];
end

%% Plot

hold on
above_equator_index = transpose(logical(pos_points(:,3)>=0));
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
axis equal
end

