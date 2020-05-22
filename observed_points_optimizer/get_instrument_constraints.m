%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

function [sun_angle_ranges, sc_angle_ranges, distance_ranges] = get_instrument_constraints(varargin)
%GET_INSTRUMENT_CONSTRAINTS Defines the observation constraints on each instrument

% Syntax: [] = get_instrument_constraints(sc_type, tolerance*)
% * Optional keyword inputs


sc_type = varargin{1};

tolerance = deg2rad(15); 
if length(varargin) > 2
    for i = 2:2:length(varargin)
        if strcmpi(varargin{i},'tolerance')
            tolerance = deg2rad(varargin{i+1});
        end
    end
end

switch sc_type 
    case 0 % Carrier Spacecraft 
        sun_angle_ranges{1} = [];
        sc_angle_ranges{1}  = [];
        distance_ranges{1}  = []; 
        
    case 1 % Imaging Spectrometer 
        sun_angles = deg2rad([0, 45]); % [rad]
        sun_angle_ranges = cell(1,length(sun_angles));
        for i = 1:length(sun_angles)
            sun_angle_ranges{i} = [sun_angles(i)-tolerance, sun_angles(i)+tolerance]; % [rad]
        end
        sc_angle_ranges{1}  = deg2rad([-5, 5]); % [rad]
        distance_ranges{1}  = [0, 50].*1000; % [m]
        
    case 2 % X Ray Spectrometer
        sun_angles = deg2rad([0, 45]); % [rad]
        sun_angle_ranges = cell(1,length(sun_angles));
        for i = 1:length(sun_angles)
            sun_angle_ranges{i} = [sun_angles(i)-tolerance, sun_angles(i)+tolerance]; % [rad]
        end
        sc_angle_ranges{1}  = deg2rad([-5, 5]); % [rad]
        distance_ranges{1}  = [0, 50].*1000; % [m]
        
    case 3 % Camera      
        sun_angles = deg2rad([0, 45]); % [rad]
        sun_angle_ranges = cell(1,length(sun_angles));
        for i = 1:length(sun_angles)
            sun_angle_ranges{i} = [sun_angles(i)-tolerance, sun_angles(i)+tolerance]; % [rad]
        end
        sc_angle_ranges{1}  = deg2rad([-10, 10]); % [rad]
        distance_ranges{1}  = [0, 50].*1000; % [m]
        
    case 4 % Altimeter
        sun_angle_ranges{1} = [-2*pi, 2*pi]; 
        sc_angle_ranges{1}  = deg2rad([-5, 5]); % [rad]
        distance_ranges{1}  = [0, 50].*1000; % [m]
                
    case -1 % A permissive instrument used for testing purposes
        sun_angle_ranges{1} = [-2*pi, 2*pi];
        sc_angle_ranges{1}  = [-2*pi, 2*pi]; % [rad]
        distance_ranges{1}  = [0, 40].*1000; % [m]
        
    case -2 % A non-permissive instrument used for testing purposes
        sun_angle_ranges{1} = [0, 0];
        sc_angle_ranges{1}  = [0, 0]; % [rad]
        distance_ranges{1}  = [0, 0].*1000; % [m]
        
    case -3 % Same as -1, but has lower reward/priority in get_coverage_reward_map
        sun_angle_ranges{1} = [-2*pi, 2*pi];
        sc_angle_ranges{1}  = [-2*pi, 2*pi]; % [rad]
        distance_ranges{1}  = [0, 40].*1000; % [m]
        
    otherwise
        error('SC type is incorrect!')
end


