function [sun_angle_ranges, sc_angle_ranges, distance_ranges, data_rate] = get_instrument_constraints(type)
%GET_INSTRUMENT_CONSTRAINTS Defines the observation constraints on each
%instrument

% WARNING: arbitrary data rates entered. Camera sun angle range has also
% been changed. 

tolerance = deg2rad(15); 

switch type 
    case 0 % Carrier Spacecraft 
        sun_angle_ranges{1} = [];
        sc_angle_ranges{1}  = [];
        distance_ranges{1}  = []; 
        data_rate = [];
    case 1 % Imaging Spectrometer 
        sun_angles = deg2rad([0, 45]); % [rad]
        sun_angle_ranges = cell(1,length(sun_angles));
        for i = 1:length(sun_angles)
            sun_angle_ranges{i} = [sun_angles(i)-tolerance, sun_angles(i)+tolerance]; % [rad]
        end
        sc_angle_ranges{1}  = deg2rad([-5, 5]); % [rad]
        distance_ranges{1}  = [30, 36].*1000; % [m]
        data_rate = 1*8e9; % [bit/s]
        
    case 2 % X Ray Spectrometer
        sun_angles = deg2rad([0, 45]); % [rad]
        sun_angle_ranges = cell(1,length(sun_angles));
        for i = 1:length(sun_angles)
            sun_angle_ranges{i} = [sun_angles(i)-tolerance, sun_angles(i)+tolerance]; % [rad]
        end
        sc_angle_ranges{1}  = deg2rad([-5, 5]); % [rad]
        distance_ranges{1}  = [30, 36].*1000; % [m]
        data_rate = 1*8e9; % [bit/s]
        
    case 3 % Camera      
        sun_angle_ranges{1} = deg2rad([-45, 45]); % [rad]   *Does not agree with science table, assuming that the value should be interpreted as ranges. 
        sc_angle_ranges{1}  = deg2rad([-10, 10]); % [rad]
        distance_ranges{1}  = [0, 40].*1000; % [m]
        data_rate = 1*8e9; % [bit/s]
        
    case 4 % Altimeter
        sun_angle_ranges{1} = [-2*pi, 2*pi]; 
        sc_angle_ranges{1}  = deg2rad([-tolerance, tolerance]); % [rad]
        distance_ranges{1}  = [47, 53].*1000; % [m]
        data_rate = 1*8e6; % [bit/s]
        
    case 5 % Radio Science
        sun_angle_ranges{1} = [-2*pi, 2*pi];
        sc_angle_ranges{1}  = deg2rad([-10, 10]); % [rad]
        distance_ranges{1}  = [0, 100].*1000; % [m]
        data_rate = 1*8e6; % [bit/s]
        
    case 6 % Magnetometer 
        sun_angle_ranges{1} = [-2*pi, 2*pi];
        sc_angle_ranges{1}  = deg2rad([-tolerance, tolerance]); % [rad]
        distance_ranges{1}  = [0, 40].*1000; % [m]
        data_rate = 1*8e6; % [bit/s]
        
end


