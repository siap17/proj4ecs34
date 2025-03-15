The CPathRouter class serves as an abstract base class for routing algorithms, helping with a unified interface for pathfinding implementations like CDijkstraPathRouter.

key functions include the following:
AddVertex(tag): inserts a new vertex and returns its unique ID.
AddEdge(src, dest, weight, bidir): establishes a weighted edge between two vertices.
FindShortestPath(src, dest, path): computes the shortest path from src to dest.
Precompute(deadline): allows precomputation for optimized path queries.

ex: 

    CPathRouter* router = new CDijkstraPathRouter();
    auto v1 = router->AddVertex("A");
    auto v2 = router->AddVertex("B");

    router->AddEdge(v1, v2, 2.0, false);

    std::vector<CPathRouter::TVertexID> path;
    router->FindShortestPath(v1, v2, path);
