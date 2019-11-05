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

function [] = plot_coverage_and_communications(varargin)
% plot_coverage_and_communications A function to plot both coverage and
% communications.
% Syntax: [] = plot_coverage_and_communications(Swarm, ErosModel, color_array*, absolute*, record_video*, videoname*)
% * Optional keyword inputs

Swarm = varargin{1};
AsteroidModel = varargin{2};
absolute= false;
color_array = ['r', 'b', 'g', 'c', 'm'];
color_array = rand(3,swarm.get_num_spacecraft());

record_video = false;

videoname = ['plot_ICC_',datestr(now,'yyyymmdd_HHMMSS'),'.avi'];

if length(varargin) > 2
    for i = 3:2:length(varargin)
        if strcmpi(varargin{i},'color_array') || strcmpi(varargin{i},'colorArray') ||  strcmpi(varargin{i},'color')
            color_array = varargin{i+1};
        end
        if strcmpi(varargin{i},'absolute')
            absolute = varargin{i+1};
        end
        if strcmpi(varargin{i},'record_video') || strcmpi(varargin{i},'video') || strcmpi(varargin{i},'save_video')
            record_video = varargin{i+1};
        end
        if strcmpi(varargin{i},'video_name') || strcmpi(varargin{i},'videoname')
            videoname = varargin{i+1};
        end
    end
end

% Initialize figure and estimate how big it should be
h1=figure();
set(h1,'Color',[1 1 1]);
set(h1,'units','normalized','outerposition',[0 0 1 1])
set(h1,'PaperPositionMode','auto');
initialize_spatial_plot_3d()
hold on 
axis equal
% Initial plot - just to get a sense of the size
plot_coverage_and_communications_frame(Swarm, AsteroidModel,length(Swarm.sample_times), 'absolute', absolute, 'figure_handle', h1, 'color_array', color_array);
V = axis();
cla;
axis(V);

for t_ix = 1:length(Swarm.sample_times)
    plot_handles = plot_coverage_and_communications_frame(Swarm, AsteroidModel,t_ix, 'absolute', absolute, 'figure_handle', h1, 'color_array', color_array);
    drawnow limitrate
    pause(0.125);
    for entry_ix = 1:length(plot_handles)
        if ~isempty(plot_handles{entry_ix})
            delete(plot_handles{entry_ix})
        end
    end
end
plot_coverage_and_communications_frame(Swarm, AsteroidModel,t_ix, 'absolute', absolute, 'figure_handle', h1, 'color_array', color_array);