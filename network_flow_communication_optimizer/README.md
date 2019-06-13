# network_flow_communication_optimizer

A set of functions implementing a network flow-based communication
optimizer. Based on the time-varying network flow model developed by 
Sam Friedman (CSM) and Federico Rossi (JPL).

The function `communication_optimizer.m` takes as inputs a set of orbits
and data production rates and (optionally) a communication model
specifying the relationship between the spacecraft positions and the 
inter-spacecraft bandwidth.
It returns the optimal set of communication flows that maximize the
amount of data delivered to the carrier (optionally weighted by data
priorities).

## Example Output
![Communication between spacecraft](../media/network_flow_comm_optimizer.gif)