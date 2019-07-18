# observed_points_optimizer
___

## Overview 
- This module determines which points on the body will be observed, given the spacecraft orbits.
- Observed points will only be computed for the spacecraft whose trajectories are initialized. 
- The observed points are computed in one of three ways depending on the value of _flag_optimization_approach_. Cases: 
    0. Observe only nadir point. Do not perform optimization. 
    1. Calls  `single_agent_points_optimizer.m` and optimizes the observed points for each spacecraft individually, and independently of one another.
    2. Calls `swarm_points_optimizer.m` which takes in all of the observable points of each spacecraft at each time sample, and decides which points will be observed. 
___

## Optmization Problem (swarm_points_optimizer)
### Assumptions: 
- Each spacecraft can observe at most one point per timestep 
- Vertices can be observed no more than once 

### Which Points are Observable?  
- `get_observable_points()` determines which points are feasible for observation (observable) by a given spacecraft in a particular configuration. 
- This function should be modified in accordance with the science constraints of the mission.
- The set of observable points for spacecraft _i_ at time _k_ is stored in the cell array _observable_points{i,k}_

### What is the Value of Observing a Particular Point? 
- `get_coverage_reward_map()` defines the value accociated with each observable point by defining a _reward_map_. 
- This function should be modified in accordance with the science constraints of the mission. - The _reward_map_ is a cell array of size _[1 x N]_ with _N_ being the number of spacecraft
    - _reward_map{ i }( j, k )_ is the value of agent _i_ observing vertex _j_ at time _k_. Note that this only needs to be defined for the observable combinations of (i,j,k) (i.e. _(i,j,k)_ | _j_ $in$ _observable_points{i,k}_)
___

## Advanced Usage
The last two input arguments _sc_optimized_ and _sc_find_observable_pts_ are optional. If specified, they do the following:
- _sc_optimized_ [vector]: Optimization will only be performed on the spacecraft in Swarm whose indicies are in this vector. 
- _sc_find_observable_points_ [vector]: The observable points will only be computed for spacecraft in Swarm whose indicies are in this vector. 
___

## Observation Type Definitions
- _sc_obs_type_ = 0: carrier spacecraft 
-- Does not observe the asteroid.
- _sc_obs_type_ = 1: instrument carrying spacecraft
-- Observes the asteroid. 
___ 

## Dependencies: 
- none