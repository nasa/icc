%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%  A function that returns basic properties of a swarm as text            %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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


function [] = evaluate_swarm(swarm)

N = swarm.get_num_spacecraft();
K = swarm.get_num_timesteps;

instruments = [];
for i=1:N
    instruments = [instruments, swarm.Parameters.types{i}];
end
instruments = unique(instruments);

% This hardcoded map corresponds to the map in instrument_reward and
% get_data_rates.
instrument_names = {'Carrier','Imaging Spectrometer ','X Ray Spectrometer','Camera','Altimeter'};

observations_count = zeros(N,1);
reward_count = zeros(N,1);

for i=1:N
    for k=1:K
        if 0~=swarm.Observation.observed_points(i,k)
            observations_count(i) = observations_count(i) + 1;
            reward_count(i) = reward_count(i) + swarm.Observation.priority(i,k);
        end
    end
end

observations_by_instrument =zeros(size(instruments));
rewards_by_instrument =zeros(size(instruments));

for i=1:N
    instrument = swarm.Parameters.types{i};
    instrument_id = find(instruments==instrument);
    observations_by_instrument(instrument_id) = observations_by_instrument(instrument_id)+observations_count(i);
    rewards_by_instrument(instrument_id) = rewards_by_instrument(instrument_id)+reward_count(i);
    fprintf("Spacecraft %d is a %s\n", i, instrument_names{instrument_id});
end

observability_by_instrument = zeros(size(rewards_by_instrument));
for instrument_id = 1:length(instruments)
    instrument = instruments(instrument_id);
    observability_by_instrument(instrument_id) = rewards_by_instrument(instrument_id)/instrument_reward(instrument);
    
end

average_reward_by_instrument = rewards_by_instrument./observations_by_instrument;
average_observability_by_instrument = observability_by_instrument./observations_by_instrument;

fprintf("Total reward: %d\n",sum(rewards_by_instrument));
fprintf("Total: %d observations, with average reward %d, total reward %d, and average `instrument-normalized` observability %d\n", ...
    sum(observations_by_instrument), ...
    sum(rewards_by_instrument)/sum(observations_by_instrument), ...
    sum(rewards_by_instrument), ...
    sum(observability_by_instrument)/sum(observations_by_instrument));
for instrument_id = 1:length(instruments)
    fprintf("Instrument %s: %d observations, with average reward %d, total reward %d, and average `instrument-normalized` observability %d\n",...
        instrument_names{instrument_id},...
        observations_by_instrument(instrument_id), ...
        average_reward_by_instrument(instrument_id), ...
        rewards_by_instrument(instrument_id), ...
        average_observability_by_instrument(instrument_id) );
end