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

function [h] = render_observed_points_3d(varargin)
%F_RENDER_OBSERVED_POINTS_3D Renders location of the observed points given
%the Asteroid and Swarm objects
%    Syntax: render_observed_points_3d(AsteroidModel, Swarm, 'color_array', *color_array, 'show_not_observed', *showNotObserved )
%    *optional input
%
%   Inputs:
%    - AsteroidModel
%    - Swarm
%
%   Outputs:
%    - h: plot handle for the points rendered

%% Interpret the Inputs
% n_inputs = max(size(varargin));
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

color_array = ['c' 'r' 'b' 'g' 'm'];
showNotObserved = false;
markersize = 3;

if nargin > 3
    for i = 4:2:n_inputs
        if strcmpi(varargin{i},'color_array') || strcmpi(varargin{i},'colorArray') ||  strcmpi(varargin{i},'color')
            color_array = varargin{i+1};
        end
        if strcmpi(varargin{i},'show_not_observed') || strcmpi(varargin{i},'showNotObserved')
            showNotObserved = varargin{i+1};
        end
        if strcmpi(varargin{i},'markersize') || strcmpi(varargin{i},'marker_size')
            markersize = varargin{i+1};
        end
    end
end


%% Render Points

if showNotObserved == true
    % Plot all of the points on the body which are not currently being observed
    not_observed_index = logical(point_index == 0);
    h(ns+1) = plot3(pos_points(not_observed_index,1),pos_points(not_observed_index,2),...
        pos_points(not_observed_index,3),'ok','MarkerFaceColor','none','MarkerSize',markersize);
end

% Plot points being observed, and colorcode by the agent number
for ns = 1:1:n_spacecraft
    observed_index = logical(point_index == ns);
    if sum(observed_index)>0
        h(ns) = plot3(pos_points(observed_index,1),pos_points(observed_index,2),...
            pos_points(observed_index,3),'o','MarkerFaceColor',color_array(:,mod(i_sc-1,size(color_array,2))+1)',...
            'MarkerEdgeColor',color_array(:,mod(i_sc-1,size(color_array,2))+1)','MarkerSize',markersize);
    end
end

end
