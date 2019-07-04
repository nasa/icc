# relay_orbit_optimizer

A set of functions for optimizing spacecraft orbits for coverage.

- `create_neighbor_set.m` processes shape model data into a cell of connected
Nodes. Nodes are said to be connected if they share a face. 

- `get_node_neighbors.m` returns the set all neighboring vertices to a given set
of vertices. Removes repreated elements.

- `get_voronoi_data.m` computes information for a discrete Voronoi tesselation
from a given set of sources using wavefront propagation. 

- `propagate_wavefront.m` propagates a "wavefront" of explored nodes over the 
small body. i.e. updates the explored verticies and boundary by exploring 
additional neighbors within a given distance. 