# monte_carlo_coverage_optimizer

## Overview 
Generates a set of random orbits, and assigns one of these orbits to each instrument carrying spacecraft in the swarm. The orbits are chosen sequentially (one spacecraft after another) such that the overall coverage reward is maximized on each iteration. 

## Observation Type Definitions
- _sc_obs_type_ = 0: carrier spacecraft 
-- Does not observe the asteroid; not assigned an orbit.
- _sc_obs_type_ = 1: instrument carrying spacecraft
-- Observes the asteroid; assigned an orbit. 

## Dependencies: 
- _observation_points_optimizer_ module 
- _small_body_dynamics_
