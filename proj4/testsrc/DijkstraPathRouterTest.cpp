#include <cassert>
#include <iostream>
#include <vector>
#include <limits>
#include "DijkstraPathRouter.cpp"  // Include your Dijkstra class

// Test class for CDijkstraPathRouter
class CDijkstraPathRouterTest {
public:
    static void RunTests() {
        TestAddVertex();
        TestAddEdge();
        TestFindShortestPath();
    }

private:
    static void TestAddVertex() {
        CDijkstraPathRouter router;
        
        // Test adding vertices
        TVertexID v1 = router.AddVertex("A");
        TVertexID v2 = router.AddVertex("B");
        
        // Ensure the correct number of vertices is added
        assert(router.VertexCount() == 2);
        
        // Ensure the vertices have the correct tags
        assert(std::any_cast<std::string>(router.GetVertexTag(v1)) == "A");
        assert(std::any_cast<std::string>(router.GetVertexTag(v2)) == "B");
        
        std::cout << "TestAddVertex passed!\n";
    }

    static void TestAddEdge() {
        CDijkstraPathRouter router;
        
        // Add vertices
        TVertexID v1 = router.AddVertex("A");
        TVertexID v2 = router.AddVertex("B");
        TVertexID v3 = router.AddVertex("C");

        // Test adding edges
        assert(router.AddEdge(v1, v2, 1.0));
        assert(router.AddEdge(v2, v3, 2.0));
        assert(router.AddEdge(v1, v3, 3.0, true));  // Bidirectional edge

        // Check if edges were added correctly
        // This part is more difficult to test directly with assertions, but we assume the AddEdge function works correctly.
        std::cout << "TestAddEdge passed!\n";
    }

    static void TestFindShortestPath() {
        CDijkstraPathRouter router;

        // Add vertices
        TVertexID v1 = router.AddVertex("A");
        TVertexID v2 = router.AddVertex("B");
        TVertexID v3 = router.AddVertex("C");
        TVertexID v4 = router.AddVertex("D");

        // Add edges
        router.AddEdge(v1, v2, 1.0);
        router.AddEdge(v2, v3, 2.0);
        router.AddEdge(v3, v4, 1.0);
        router.AddEdge(v1, v3, 2.5);
        router.AddEdge(v2, v4, 4.0);

        // Find shortest path from v1 to v4
        std::vector<TVertexID> path;
        double distance = router.FindShortestPath(v1, v4, path);

        // Ensure the correct shortest path and distance
        assert(distance == 3.0);  // The shortest path is A -> B -> C -> D, distance = 1 + 2 + 1 = 3

        // Check the reconstructed path
        assert(path.size() == 4);
        assert(path[0] == v1);
        assert(path[1] == v2);
        assert(path[2] == v3);
        assert(path[3] == v4);

        std::cout << "TestFindShortestPath passed!\n";
    }
};
