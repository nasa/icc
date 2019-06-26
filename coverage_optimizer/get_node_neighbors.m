function unique_new_neighbors = get_node_neighbors(vertices , neighbor_set)

sort = false; 

new_neighbors = []; % neighbors of first vertex
for i = 1:length(vertices)
    new_neighbors = [new_neighbors, neighbor_set{vertices(i)}];
end
if sort==true
    unique_new_neighbors = sort(unique(new_neighbors));
else
    unique_new_neighbors = unique(new_neighbors);
end
end


