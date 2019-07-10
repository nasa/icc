# observed_points_optimizer

## Overview 
Given a set of orbits and a map of the currently observed points on the asteroid, _observed_points_optimizer_ determines which points on the asteroid will be observed. This is refered to as an _optimizer_ since the observed points are (ideally) selected from a set of feaible points for observation in such a way that the overall observation reward is maximized. 

## Supporting Functions
- `get_coverage_reward()` defines the value accociated with a given set of observation points
- `get_observable_points()` determines which points are feasible for observation by a given spacecraft in a particular configuration. 

## Observation Type Definitions
- _sc_obs_type_ = 0: carrier spacecraft 
-- Does not observe the asteroid.
- _sc_obs_type_ = 1: instrument carrying spacecraft
-- Observes the asteroid. 

## Dependencies: 
- none