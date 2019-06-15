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
%    Syntax: render_observed_points_2d(pos_points, point_index, n_spacecraft, above_or_below, 'color_array', *color_array)
%    *optional input
%
%   Inputs:
%    - pos_points [km]: [N_VERTICIES x 3] Array of model vertex points
%    - point_index: Vector of indicies indicating the observed points on
%       the asteroid
%    - n_spacecraft: Number of spacecraft
%    - above_or_below: {'above','below'} Side of the asteroid to be viewed
%       in the 2D projection
%    - *color_array: Array of colors assigned to the spacecraft
%
%   Outputs:
%    - h: plot handle for the points rendered

%% Interpret the Inputs

n_inputs = max(size(varargin));
pos_points = varargin{1} ;  
point_index = varargin{2} ;
n_spacecraft = varargin{3} ;
above_or_below = varargin{4};

colorSpecified = false;

if n_inputs > 4
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
axis equal
end

