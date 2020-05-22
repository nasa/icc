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

function data_rate = get_data_rates(varargin)
% GET_DATA_RATES Takes in sc_type, AsteroidModel, optionally observed_points
% and returns the data_rate vector

sc_type = varargin{1};

AsteroidModel = varargin{2};

flag_observed_point_given = 0;
if length(varargin) > 3
    observed_points = varargin{3};
    flag_observed_point_given = 1;
end


switch sc_type 
    case 0 % Carrier Spacecraft 
        data_per_metersq = 0;
        
    case 1 % Imaging Spectrometer 
        resolution_on_surface = 1; % [pixel per meter square]
        data_per_pixel = 360*16; % [bits per pixel]
        data_per_metersq = data_per_pixel*resolution_on_surface; % [bits per meter square]
        
    case 2 % X Ray Spectrometer
        resolution_on_surface = 1/(10*10); % [pixel per meter square]
        data_per_pixel = 60*16; % [bits per pixel]
        data_per_metersq = data_per_pixel*resolution_on_surface; % [bits per meter square]
        
    case 3 % Camera      
        resolution_on_surface = 1/(0.1*0.1); % [pixel per meter square]
        data_per_pixel = 3*16; % [bits per pixel]
        data_per_metersq = data_per_pixel*resolution_on_surface; % [bits per meter square]
        
    case 4 % Altimeter
        resolution_on_surface = 1/(0.1*0.1); % [pixel per meter square]
        data_per_pixel = 1*16; % [bits per pixel]
        data_per_metersq = data_per_pixel*resolution_on_surface; % [bits per meter square]
                
    case -1 % A permissive instrument used for testing purposes
        data_per_metersq = 1; % [bits]
        
    case -2 % A non-permissive instrument used for testing purposes
        data_per_metersq = 1; % [bits]
        
    case -3 % Same as -1, but has lower reward/priority in get_coverage_reward_map
        data_per_metersq = 1; % [bits]
        
    otherwise
        error('SC type is incorrect!')
end

if sc_type < 0
    data_rate = data_per_metersq;
else
    if (flag_observed_point_given == 0)
        mean_facetArea = mean(AsteroidModel.BodyModel.shape.facetArea)*1e6; % [meter square]
        data_rate = data_per_metersq*mean_facetArea; % [bits]
        
    else
        % observed_points is given
        
        data_rate = zeros(1,length(observed_points));
        
        flag_positive_observed_points = logical(observed_points > 0);
        facetArea_vector = (1e6)*AsteroidModel.BodyModel.shape.facetArea(flag_positive_observed_points); % [meter square]
        data_rate(1,flag_positive_observed_points) = data_per_metersq*facetArea_vector; % [bits]
    end
end


