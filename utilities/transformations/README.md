# Transformations 

- op2rv: convert [Keplerian orbital elements](https://en.wikipedia.org/wiki/Orbital_elements) to radius and velocity. 

- rv2op: converts radius and velocity to Keplerian orbital elements. Suffers from singularities for circular or equatorial orbits.

- rotmat: generate rotation matrices from given Euler angles.

- rotation_matrix_at_t: generate rotation matrix from simulation time