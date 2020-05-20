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

function data_rate_Magnetometer_RadioScience = get_instrument_constraints_Magnetometer_RadioScience(Swarm)
% GET_INSTRUMENT_CONSTRAINTS_MAGNETOMETER_RADIOSCIENCE 
% Defines the constant Magnetometer and Radio Science observation done by
% all spacecraft all the time

data_rate_Magnetometer = 48; % [bits]

data_rate_RadioScience = 192; % [bits]

data_rate_Magnetometer_RadioScience = (Swarm.sample_times(2)-Swarm.sample_times(1))*(data_rate_Magnetometer + (Swarm.get_num_spacecraft-1)*data_rate_RadioScience); % [bits]