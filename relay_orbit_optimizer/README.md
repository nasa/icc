# relay_orbit_optimizer

A set of functions implementing a gradient-based optimizer for relay orbits.

The function `relay_optimization_driver.m` creates a simple problem with two
spacecraft collecting data and attempts to optimize the orbit of a single relay
spacecraft so as to maximize the amount of data delivered to the carrier.

We plan to provide a generic interface that takes as inputs a set of 
science spacecraft and a set of relays and returns the optimal relay orbits.
The interface and implementation are a work in progress.