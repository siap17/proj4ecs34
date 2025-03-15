
The CDijkstraPathRouter class implements Dijkstra’s algorithm to compute the shortest paths between nodes in a graph. It efficiently handles weighted graphs and supports bidirectional edges.

In lines 4-8 we define the SImplementation struct, which holds internal data structures.vertices maps vertex IDs to arbitrary tags.
adjacencyList stores graph edges with weights. NoPathExists represents unreachable nodes using infinity.
In lines 10-14 the constructor and destructor handle resource management. The unique pointer DImplementation ensures memory safety.
In lines 16-23 we use the following:
VertexCount(): ret the total number of vertices in the graph.
AddVertex(tag): adds a new vertex with a unique ID and assigns it a tag.
In lines 25-32 we have GetVertexTag(id) which retrieves the tag of a given vertex. If the vertex does not exist, it returns an empty std::any value.
lines 34-44 have AddEdge(src, dest, weight, bidir) which adds a weighted edge between two vertices. If bidir is true, it also creates the reverse edge. it snsures edges are valid and do not have negative weights.
Lines 46-49 have Precompute(deadline)- a placeholder function for preprocessing. Currently, it does nothing and always returns true.
in lines 51-92 we have the following:
FindShortestPath(src, dest, path): implements Dijkstra’s algorithm using a priority queue and initializes all nodes with infinite distance, and also uses a min-heap (priority queue) to efficiently expand the shortest path. If a path exists, reconstructs the shortest route and returns the total cost. o/w it clears the path and returns NoPathExists.


ex usage: 
    CDijkstraPathRouter router;
    auto v1 = router.AddVertex("Start");
    auto v2 = router.AddVertex("End");
    router.AddEdge(v1, v2, 3.5, true);
    std::vector<CDijkstraPathRouter::TVertexID> path;
    double cost = router.FindShortestPath(v1, v2, path);
    std::cout << "Shortest path cost: " << cost << std::endl;
